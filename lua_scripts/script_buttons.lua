--[[
Monitor joystick buttons and send STATUSTEXT messages.

This will map the script buttons to the _shifted_ XBox dpad buttons:

BTN11_SFUNCTION  109  # shift-dpad-up       script_2
BTN12_SFUNCTION  110  # shift-dpad-down     script_3
BTN13_SFUNCTION  111  # shift-dpad-left     script_4
BTN14_SFUNCTION  108  # shift-dpad-right    script_1
]]--

local UPDATE_MS = 100

function update()

  for i = 1, 4 do
    if sub:get_and_clear_button_count(i) > 0 then
      gcs:send_text(6, string.format("script button %d", i))
    end
  end

  return update, UPDATE_MS
end

gcs:send_text(6, "script_buttons.lua running")

return update()