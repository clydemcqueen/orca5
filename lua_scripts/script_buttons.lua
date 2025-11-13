--[[
Monitor joystick buttons and send STATUSTEXT messages.

This will map the script buttons to the _shifted_ XBox controller buttons A, B, X, Y:
param set BTN0_SFUNCTION 108
param set BTN1_SFUNCTION 109
param set BTN2_SFUNCTION 110
param set BTN3_SFUNCTION 111
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