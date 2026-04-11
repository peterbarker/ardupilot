-- warn the user if wind speed exceeds a threshold, failsafe if a second threshold is exceeded

-- note that this script is only intended to be run on ArduPlane

-- note that the failsafe action is one-shot - it only ever triggers
-- RTL once and the script then exits

-- tuning parameters
local warn_speed = 10 -- metres/second
local failsafe_speed = 15 -- metres/second
local warning_interval_ms = uint32_t(15000) -- send user message every 15s

local warning_last_sent_ms = uint32_t() -- time we last sent a warning message to the user

local AHRS_WIND_MAX = Parameter('AHRS_WIND_MAX')
local AHRS_WIND_MAX_previous_value = AHRS_WIND_MAX:get()

local ARSPD_OPTIONS = Parameter('ARSPD_OPTIONS')
local ARSPD_OPTIONS_previous_value = ARSPD_OPTIONS:get()

-- threshold values for determining if GPS data is good:
-- set any of these to zero to disable the check
local GPS_MAX_GROUND_SPEED_MS = 40
local GPS_MIN_SATELLITES = 6
local GPS_MAX_HDOP_M = 20
local GPS_MAX_SPEED_ACCURACY_MS = 10

function configure_for_bad_gps_data()
   AHRS_WIND_MAX_previous_value = AHRS_WIND_MAX:get()
   AHRS_WIND_MAX:set(0)  -- disable wind max check

   ARSPD_OPTIONS_previous_value = ARSPD_OPTIONS:get()
   -- turn off bit 0 (ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE)
   local new_value = ARSPD_OPTIONS_previous_value & ~(1<<0)
   ARSPD_OPTIONS:set(new_value)
end

function configure_for_good_gps_data()
   -- we do not revert AHRS_WIND_MAX if some other mechanism has
   -- fiddled with it
   if AHRS_WIND_MAX:get() == 0 then
      AHRS_WIND_MAX:set(AHRS_WIND_MAX_previous_value)  -- re/instate WIND_MAX check
   end
   -- turn on bit 0 (ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE)
   ARSPD_OPTIONS_new_value = ARSPD_OPTIONS_previous_value | ~(1<<0)
   ARSPD_OPTIONS:set(ARSPD_OPTIONS_new_value)
end

function reconfigure_based_on_gps_health()
    -- check GPS health.  If the GPS is unhealthy (as esimated via
    -- satellite count / HDOP) then we should not validate the
    -- airspeed sensor data against the GPS velocity
    if gps_data_likely_bad() then
       if configured_for_bad_gps_data then
          -- we have already changed the vehicle config
          return
       end
       configure_for_bad_gps_data()
       configured_for_bad_gps_data = true
       return
    end

    -- GPS data is currently likely good
    if not configured_for_bad_gps_data then
       -- nothing to do
       return
    end

    configure_for_good_gps_data()
    configured_for_bad_gps_data = false
end

function gps_data_likely_bad()
   -- we check all GPSs.  Either having bad data is enough for us to get unhappy
   for i=0, 1 do
      if GPS_MAX_GROUND_SPEED_MS > 0 and gps:ground_speed(i) > GPS_MAX_GROUND_SPEED_MS then
         return true
      end
      if GPS_MIN_SATELLITES and gps:num_sats(i) < GPS_MIN_SATELLITES then
         return true
      end
      if GPS_MAX_HDOP_M > 0 and gps:horizontal_accuracy(i) > GPS_MAX_HDOP_M then
         return true
      end
      if GPS_MAX_SPEED_ACCURACY_MS > 0 and gps:speed_accuracy(i) > GPS_MAX_SPEED_ACCURACY_MS then
         return true
      end
   end
   -- GPS data looks good
   return false
end

function update()

    -- adjust parameters and state if GPS data looks bad:
    reconfigure_based_on_gps_health()

    local wind = ahrs:wind_estimate() -- get the wind estimate
    if wind and not configured_for_bad_gps_data then
        -- make a 2D wind vector
        wind_xy = Vector2f()
        wind_xy:x(wind:x())
        wind_xy:y(wind:y())
        speed = wind_xy:length() -- compute the wind speed
        if speed > failsafe_speed then
            gcs:send_text(0, "Wind failsafe at " .. speed .. " metres/second")
            vehicle:set_mode(11) -- FIXME: should be an enum.  11 is RTL.
            return
        end
        if speed > warn_speed then
            if millis() - warning_last_sent_ms > warning_interval_ms then
                    gcs:send_text(4, "Wind warning at " .. speed .. " metres/second")
                warning_last_sent_ms = millis()
            end
        end
    end

    return update, 1000
end

-- initially configure on the assumption that the GPS data is good.
-- In case we powered off configured for bad data
configure_for_good_gps_data()

return update, 1000
