-- BlackBox logging for f3a flights

gcs:send_text(0, "BlackBox Logging v0.1") -- send the traditional message

local PARAM_TABLE_KEY = 72

-- update parameters
assert(param:add_table(PARAM_TABLE_KEY, "BBOX_", 2), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'SPD', 5), 'could not add SPD param')

local param_speed = Parameter("BBOX_SPD")

last_over_speed_ms = 0

function update()

  gps_speed = gps:speed_accuracy(gps:primary_sensor())

  over_speed = gps_speed > param_speed

  now_ms = millis()
  if over_speed then
    last_over_speed_ms = now
  end

  if arming:is_armed() then
    if now - last_overspeed_ms > 20000 then
      arm:disarm()
    end
  elseif over_speed then
    arm:arm()
  end

  gcs:send_text(0, "BlackBox: %f", gps_speed)
  return update, 1000 -- reschedules the loop
end

return update, 10000
