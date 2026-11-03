--[[
    Use GUIDED mode to execute a repeating out-and-back maneuver at constant depth and heading.

    Works with ArduSub 4.7.2.

    Behavior when entering GUIDED mode:
    1. Waits for valid EKF relative position and velocity.
    2. Captures starting position (NED) and heading (yaw).
    3. Loops continuously while in GUIDED mode:
       - OUTBOUND: moves OAB_DIST meters forward along initial heading at OAB_SPD.
       - PAUSE_OUT: holds position at outbound waypoint for OAB_PAUSE seconds.
       - INBOUND: moves backward along initial heading back to starting position at OAB_SPD.
       - PAUSE_START: holds position at starting waypoint for OAB_PAUSE seconds, then repeats.
    4. Exiting GUIDED mode (or disarming) immediately stops and resets the controller.

    Custom OAB parameters:
    * OAB_DIST  sets the outbound distance in meters (default 5.0 m)
    * OAB_SPD   sets the cruise speed during GUIDED mode in m/s (default 0.2 m/s)
    * OAB_PAUSE sets the pause duration at each waypoint in seconds (default 3.0 s)
    * OAB_TOL   sets the waypoint acceptance radius in meters (default 0.35 m)
]]--

local PARAM_TABLE_KEY = 97
local PARAM_TABLE_PREFIX = "OAB_"

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), "out_and_back: could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "DIST", 5.0), "out_and_back: could not add DIST param")
assert(param:add_param(PARAM_TABLE_KEY, 2, "SPD", 0.2), "out_and_back: could not add SPD param")
assert(param:add_param(PARAM_TABLE_KEY, 3, "PAUSE", 3.0), "out_and_back: could not add PAUSE param")
assert(param:add_param(PARAM_TABLE_KEY, 4, "TOL", 0.35), "out_and_back: could not add TOL param")

local oab_dist_p = Parameter("OAB_DIST")
local oab_spd_p = Parameter("OAB_SPD")
local oab_pause_p = Parameter("OAB_PAUSE")
local oab_tol_p = Parameter("OAB_TOL")
local wp_spd_p = Parameter("WP_SPD")

local RUN_HZ = 20                   -- update frequency (20 Hz)
local MAX_DT = 0.5                  -- cap dt to avoid large integration step on stall
local ZERO_VEC = Vector3f()

local GUIDED_MODE_NUM = 4           -- sub GUIDED mode number

-- Controller configuration
local ACC_XY = 0.5                  -- horizontal acceleration limit in m/s^2
local SPEED_MIN = 0.05              -- minimum cruise speed in m/s

-- State machine constants
local STATE_IDLE = 0
local STATE_OUTBOUND = 1
local STATE_PAUSE_OUT = 2
local STATE_INBOUND = 3
local STATE_PAUSE_START = 4

-- Controller state variables
local prev_mode
local guided_active = false
local state = STATE_IDLE
local cycle_count = 0
local last_time_ms = millis()
local pause_start_ms = 0
local last_prereq_warn_ms = 0

local start_x = 0
local start_y = 0
local start_z = 0
local start_yaw_rad = 0
local start_yaw_deg = 0
local dir_x = 1
local dir_y = 0
local leg_dist = 5.0
local out_x = 0
local out_y = 0

local track_pos = 0
local track_vel = 0
local commanded_speed = 0
local dist_to_wp = 0

local pos_target = Vector3f()       -- 3D position target in m (NED)
local vel_target = Vector3f()       -- 3D velocity target in m/s (NED)
local acc_target = Vector3f()       -- 3D acceleration target in m/s^2 (NED)

local function clamp(val, min_val, max_val)
  if val < min_val then return min_val end
  if val > max_val then return max_val end
  return val
end

-- Clamp initial OAB_SPD if it exceeds WP_SPD
local init_cruise_speed = oab_spd_p:get()
if init_cruise_speed > wp_spd_p:get() then
  oab_spd_p:set(wp_spd_p:get())
end

local function reset_guided_controller()
  guided_active = false
  state = STATE_IDLE
  cycle_count = 0
  track_pos = 0
  track_vel = 0
  commanded_speed = 0
  dist_to_wp = 0
  last_prereq_warn_ms = 0

  pos_target:x(0)
  pos_target:y(0)
  pos_target:z(0)
  vel_target:x(0)
  vel_target:y(0)
  vel_target:z(0)
  acc_target:x(0)
  acc_target:y(0)
  acc_target:z(0)
  poscontrol:set_posvelaccel_offset(ZERO_VEC, ZERO_VEC, ZERO_VEC)
end

local function step_leg(origin_x, origin_y, dest_x, dest_y, ux, uy, cur_x, cur_y, dt)
  local cruise_speed = clamp(oab_spd_p:get(), SPEED_MIN, wp_spd_p:get())
  local rem = math.max(0.0, leg_dist - track_pos)
  local stopping_speed = math.sqrt(2.0 * ACC_XY * rem)
  local desired_speed = math.min(cruise_speed, stopping_speed)

  local dv = clamp(desired_speed - track_vel, -ACC_XY * dt, ACC_XY * dt)
  local new_vel = math.max(0.0, track_vel + dv)
  local track_acc = dv / dt

  track_pos = track_pos + 0.5 * (track_vel + new_vel) * dt
  if track_pos >= leg_dist then
    track_pos = leg_dist
    new_vel = 0.0
    track_acc = 0.0
  end
  track_vel = new_vel
  commanded_speed = track_vel

  pos_target:x(origin_x + ux * track_pos)
  pos_target:y(origin_y + uy * track_pos)
  pos_target:z(start_z)

  vel_target:x(ux * track_vel)
  vel_target:y(uy * track_vel)
  vel_target:z(0)

  acc_target:x(ux * track_acc)
  acc_target:y(uy * track_acc)
  acc_target:z(0)

  local dx = dest_x - cur_x
  local dy = dest_y - cur_y
  dist_to_wp = math.sqrt(dx * dx + dy * dy)

  local tol = math.max(0.05, oab_tol_p:get())
  return (track_pos >= leg_dist) and (dist_to_wp <= tol)
end

local function hold_waypoint(wx, wy, cur_x, cur_y)
  commanded_speed = 0
  pos_target:x(wx)
  pos_target:y(wy)
  pos_target:z(start_z)
  vel_target:x(0)
  vel_target:y(0)
  vel_target:z(0)
  acc_target:x(0)
  acc_target:y(0)
  acc_target:z(0)

  local dx = wx - cur_x
  local dy = wy - cur_y
  dist_to_wp = math.sqrt(dx * dx + dy * dy)
end

local function update_guided_mode(dt)
  local now_ms = millis()
  local current_pos = ahrs:get_relative_position_NED_origin()
  local current_vel = ahrs:get_velocity_NED()

  if not guided_active then
    local send_prereq_warn = (prev_mode ~= GUIDED_MODE_NUM) or ((now_ms - last_prereq_warn_ms):tofloat() > 4000)

    if not current_pos or not current_vel then
      if send_prereq_warn then
        last_prereq_warn_ms = now_ms
        gcs:send_text(4, "out_and_back: waiting for EKF relative position")
      end
      return
    end

    start_x = current_pos:x()
    start_y = current_pos:y()
    start_z = current_pos:z()
    start_yaw_rad = ahrs:get_yaw_rad()
    start_yaw_deg = math.deg(start_yaw_rad)
    dir_x = math.cos(start_yaw_rad)
    dir_y = math.sin(start_yaw_rad)

    leg_dist = math.max(0.5, oab_dist_p:get())
    out_x = start_x + leg_dist * dir_x
    out_y = start_y + leg_dist * dir_y

    track_pos = 0
    track_vel = 0
    commanded_speed = 0
    dist_to_wp = leg_dist
    cycle_count = 0
    state = STATE_OUTBOUND

    pos_target:x(start_x)
    pos_target:y(start_y)
    pos_target:z(start_z)
    vel_target:x(0)
    vel_target:y(0)
    vel_target:z(0)
    acc_target:x(0)
    acc_target:y(0)
    acc_target:z(0)

    poscontrol:set_posvelaccel_offset(ZERO_VEC, ZERO_VEC, ZERO_VEC)

    guided_active = true
    local cruise_speed = clamp(oab_spd_p:get(), SPEED_MIN, wp_spd_p:get())
    gcs:send_text(
      6,
      string.format("out_and_back: GUIDED active, dist %.1fm, speed %.2fm/s", leg_dist, cruise_speed)
    )
  end

  if not current_pos then
    return
  end

  local cur_x = current_pos:x()
  local cur_y = current_pos:y()
  local pause_ms = math.max(0.0, oab_pause_p:get()) * 1000.0

  if state == STATE_OUTBOUND then
    if step_leg(start_x, start_y, out_x, out_y, dir_x, dir_y, cur_x, cur_y, dt) then
      hold_waypoint(out_x, out_y, cur_x, cur_y)
      state = STATE_PAUSE_OUT
      pause_start_ms = now_ms
      gcs:send_text(6, string.format("out_and_back: reached outbound (%.2fm), pausing", dist_to_wp))
    end
  elseif state == STATE_PAUSE_OUT then
    hold_waypoint(out_x, out_y, cur_x, cur_y)
    if (now_ms - pause_start_ms):tofloat() >= pause_ms then
      track_pos = 0
      track_vel = 0
      state = STATE_INBOUND
      gcs:send_text(6, "out_and_back: returning to start")
    end
  elseif state == STATE_INBOUND then
    if step_leg(out_x, out_y, start_x, start_y, -dir_x, -dir_y, cur_x, cur_y, dt) then
      hold_waypoint(start_x, start_y, cur_x, cur_y)
      state = STATE_PAUSE_START
      pause_start_ms = now_ms
      cycle_count = cycle_count + 1
      gcs:send_text(
        6,
        string.format("out_and_back: reached start (%.2fm), cycle %d complete", dist_to_wp, cycle_count)
      )
    end
  elseif state == STATE_PAUSE_START then
    hold_waypoint(start_x, start_y, cur_x, cur_y)
    if (now_ms - pause_start_ms):tofloat() >= pause_ms then
      leg_dist = math.max(0.5, oab_dist_p:get())
      out_x = start_x + leg_dist * dir_x
      out_y = start_y + leg_dist * dir_y
      track_pos = 0
      track_vel = 0
      state = STATE_OUTBOUND
      gcs:send_text(6, string.format("out_and_back: starting outbound (cycle %d)", cycle_count + 1))
    end
  end

  vehicle:set_target_posvelaccel_NED(pos_target, vel_target, acc_target, true, start_yaw_deg, false, 0, false)
end

local function log_oab_state(current_mode)
  local current_pos = ahrs:get_relative_position_NED_origin()
  local pos_x = current_pos and current_pos:x() or 0
  local pos_y = current_pos and current_pos:y() or 0
  local pos_z = current_pos and current_pos:z() or 0
  local targ_x = guided_active and pos_target:x() or 0
  local targ_y = guided_active and pos_target:y() or 0
  local targ_z = guided_active and pos_target:z() or 0

  logger:write("OAB",
    "Mode,State,Cyc,PX,PY,PZ,TX,TY,TZ,Dist,Spd",
    "BBBffffffff",
    current_mode,
    state,
    cycle_count,
    pos_x,
    pos_y,
    pos_z,
    targ_x,
    targ_y,
    targ_z,
    dist_to_wp,
    commanded_speed
  )
end

local function update()
  local now_ms = millis()
  local dt = (now_ms - last_time_ms):tofloat() / 1000.0
  if dt <= 0 then
    return update, 1000 / RUN_HZ
  end
  if dt > MAX_DT then
    dt = MAX_DT
  end
  last_time_ms = now_ms

  if not arming:is_armed() then
    if guided_active then
      reset_guided_controller()
    end
    return update, 1000 / RUN_HZ
  end

  local current_mode = vehicle:get_mode()

  if current_mode == GUIDED_MODE_NUM then
    update_guided_mode(dt)
  else
    if guided_active then
      gcs:send_text(6, "out_and_back: exited GUIDED mode, stopping")
      reset_guided_controller()
    end
  end

  log_oab_state(current_mode)

  prev_mode = current_mode
  return update, 1000 / RUN_HZ
end

gcs:send_text(6, "out_and_back.lua loaded")
return update, 1000
