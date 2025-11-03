import orca_msgs.msg
import pymavlink.dialects.v20.ardupilotmega as apm

import geometry


class Sub:
    """ArduSub state"""

    FILTER_TIMEOUT_S = 3.0

    # Listen to these MAVLink messages
    MAVLINK_MSGS = [
        'ATTITUDE',  # Orientation, always available
        'EKF_STATUS_REPORT',  # Filter status
        'GLOBAL_POSITION_INT',  # Altitude above home, use if filter_status == ALT_ONLY
        'LOCAL_POSITION_NED',  # Position, use if filter_status == HORIZ_REL
        'RANGEFINDER',  # Sonar rangefinder
    ]

    def __init__(self):
        self.last_heartbeat_s = 0.0
        self.ekf_status_report = orca_msgs.msg.FilterStatus()  # Empty message
        self.ekf_status_time: float | None = None
        self.t_map_base_ned = geometry.Pose()
        self.sonar_rf_distance = None

    def ekf_const_pos(self):
        return self.ekf_status_report.flags & apm.EKF_CONST_POS_MODE == apm.EKF_CONST_POS_MODE

    def ekf_horiz_rel(self):
        return self.ekf_status_report.flags & apm.EKF_POS_HORIZ_REL == apm.EKF_POS_HORIZ_REL

    def ekf_other(self):
        return not self.ekf_const_pos() and not self.ekf_horiz_rel()

    def update(self, conn, now_s: float, logger):
        # The timer fires at 10 Hz; send heartbeat at 1 Hz
        if now_s - self.last_heartbeat_s > 1.0:
            conn.mav.heartbeat_send(apm.MAV_TYPE_ONBOARD_CONTROLLER, apm.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_heartbeat_s = now_s

        # Drain all MAVLink messages and gather ArduSub state
        while True:
            msg = conn.recv_match(type=Sub.MAVLINK_MSGS, blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()
            if msg_type == 'ATTITUDE':
                self.t_map_base_ned.set_euler(msg.roll, msg.pitch, msg.yaw)
            elif msg_type == 'EKF_STATUS_REPORT':
                old_flags = self.ekf_status_report.flags
                self.ekf_status_report = msg
                self.ekf_status_time = now_s
                if old_flags != self.ekf_status_report.flags:
                    logger.info(f'EKF status change {old_flags} => {self.ekf_status_report.flags}, const_pos={self.ekf_const_pos()}, horiz_rel={self.ekf_horiz_rel()}')
            elif msg_type == 'GLOBAL_POSITION_INT':
                if self.ekf_const_pos():
                    self.t_map_base_ned.set_altitude(-msg.relative_alt / 1e3)  # Height above home, flip sign for NED
            elif msg_type == 'LOCAL_POSITION_NED':
                self.t_map_base_ned.set_position(msg.x, msg.y, msg.z)
            elif msg_type == 'RANGEFINDER':
                # Hack to support testing
                self.sonar_rf_distance = msg.distance if msg.distance > 0.0 else 1.0

        if self.ekf_status_time is not None and now_s - self.ekf_status_time > Sub.FILTER_TIMEOUT_S:
            logger.warn('EKF timeout')
            self.ekf_status_report = orca_msgs.msg.FilterStatus()
            self.ekf_status_time = None
