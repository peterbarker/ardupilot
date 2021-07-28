-- hit waypoints at a specified time in UTC

-- Plane only
-- a point moves linearly between from origin waypoint to target waypoint, the Plane tries to keep up with it.  Consider it a Pace Car.
-- Scripting messaging is used to feed information to the script, messages are defined below
-- script is only active when vehicle is heading towards most recently configured waypoint

-- scripting message 1:

local update_interval_ms = 100    -- script updates at 10hz
local max_mission_message_age_ms = 1000

-- vehicle performance contraints
local groundspeed_min = 10  -- minimum commanded groundspeed (m/s)
local groundspeed_max = 20  -- maximum commanded groundspeed (m/s)

-- random script constants:
local MODE_AUTO = 10
local MS_PER_WEEK = 86400*7*1000

-- persistent script state
local my_wp = nil  -- waypoint we are currently configured to head towards
local wp_target_arrival_time_ms = nil -- time in ms past system boot time we want to arrive at wp
local last_wp
local distance_between_waypoints
local new_commanded_groundspeed = -1

local last_update_ms = 0

function notify(text)
   gcs:send_text(0, "TW: " .. text)
end

function handle_mission_message(p1, p2, p3, p4)
   local gps_week = gps:time_week(gps:primary_sensor())
   local gps_week_ms = gps:time_week_ms(gps:primary_sensor())

   delta_ms = (p2 - gps_week) * 86400 + (p3 - gps_week_ms)
   notify(string.format("delta_ms=%s", tostring(delta_ms)))

   if delta_ms < 0 then
      notify(string.format("time in past"))
      return
   end
      
   my_wp = p1
   wp_target_arrival_time_ms = millis() + delta_ms

end

function update2()
     return update, update_interval_ms -- reschedules the loop
end

function update()

  -- consume a mission message first-thing:
  now = millis()
  local time_ms, p1, p2, p3, p4 = mission_receive()
  if time_ms then
     if now - time_ms < max_mission_message_age_ms then
         notify(string.format("Command: %i, %0.2f, %0.2f, %0.2f", time_ms:tofloat(), p1, p2, p3, p4))
	 handle_mission_message(p1, p2, p3, p4)  -- this may fail!
      end
  end 

  -- nudge demanded groundspeed
  if (tonumber(new_commanded_groundspeed) ~= nil and
      new_commanded_groundspeed ~= old_commanded_groundspeed) then
--     notify(string.format("Setting groundspeed to %f", tonumber(new_commanded_groundspeed)))
     vehicle:set_groundspeed_min(tonumber(new_commanded_groundspeed))
  end

  -- new_commanded_groundspeed must be updated each loop or we stop
  -- controlling the vehicle:
  old_commanded_groundspeed = new_commanded_groundspeed
  new_commanded_groundspeed = -1

  -- do nothing if not in auto mode:
  if not (vehicle:get_mode() == MODE_AUTO) then
--     notify(string.format("mode is not auto", index, distance))
     return update, update_interval_ms -- reschedules the loop
  end

  local current_wp = mission:get_current_nav_index()
  local distance_remaining = vehicle:get_wp_distance_m()

  -- current_wp and distance_remaining can come back nil even in auto mode:
  if current_wp == nil or distance_remaining == nil then
     return update, update_interval_ms -- reschedules the loop
  end

  if my_wp == nil then
     return update, update_interval_ms -- reschedules the loop
  end

  if not (my_wp == current_wp) then
     -- not heading towards my currently configured waypoint, do nothing
     return update, update_interval_ms -- reschedules the loop
  end


  if now - last_update_ms < 5000 then
     new_commanded_groundspeed = old_commanded_groundspeed
     return update, update_interval_ms -- reschedules the loop
  end
     
  -- last_update_ms = now

  if now > wp_target_arrival_time_ms then
     new_commanded_groundspeed = groundspeed_max
     return update, update_interval_ms -- reschedules the loop
  end

  time_remaining_ms = wp_target_arrival_time_ms - now

  -- notify(string.format("delta=%s", tostring(time_remaining_ms)))
  time_remaining_s = time_remaining_ms:tofloat()/1000.0

  ground_speed = gps:ground_speed(gps:primary_sensor())

  new_commanded_groundspeed = distance_remaining / time_remaining_s
  notify(string.format("dr=%0.2f tr=%0.2f gs=%0.2f ncg=%0.2f", distance_remaining, time_remaining_s, ground_speed, new_commanded_groundspeed))

  if new_commanded_groundspeed > groundspeed_max then
     new_commanded_groundspeed = groundspeed_max
  elseif new_commanded_groundspeed < groundspeed_min then
     new_commanded_groundspeed = groundspeed_min
  end
     
  -- notify(string.format("wp=%u dist=%f my_wp=%u ncg=%s", current_wp, distance_remaining, my_wp, tostring(new_commanded_groundspeed)))

  return update, update_interval_ms -- reschedules the loop
end

return update()
