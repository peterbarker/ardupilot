-- this script based on the analog_input_and_GPIO.lua script from the
--   ArduPilot repository.

-- for these examples BRD_PWM_COUNT must be 0

pin = 53  -- AUX4

gpio:pinMode(pin,0)  -- input

local old_value = false

local last_emit_ms = 0

function update()

  new_value = gpio:read(pin)

  now = millis()

  emit = not (new_value == old_value)

  -- emit unconditionally periodically:
  if (now - last_emit_ms) > 1000 then
     emit = true
  end

--  if old_value and not new_value then
--      gcs:send_text(0, string.format("GPS Jamming cleared"))
--  elseif not old_value and new_value then
--      gcs:send_text(0, string.format("GPS Jamming detected"))
--  end

  if emit then
      send_value = 0
      if new_value then
         send_value = 1
      end
      gcs:send_named_float("OA_GPS_JAM", send_value)
      last_emit_ms = now
  end

  old_value = new_value

  return update, 1000
end

return update() -- run immediately before starting to reschedule
