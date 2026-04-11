-- warn the user if wind speed exceeds a threshold, failsafe if a second threshold is exceeded

-- note that this script is only intended to be run on ArduPlane

-- note that the failsafe action is one-shot - it only ever triggers
-- RTL once and the script then exits

local PARAM_TABLE_KEY = 139
assert(param:add_table(PARAM_TABLE_KEY, "WFS_", 7), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'WARN_SPD',    10), 'could not add WFS_WARN_SPD')    -- wind warning speed (m/s)
assert(param:add_param(PARAM_TABLE_KEY, 2, 'FS_SPD',      15), 'could not add WFS_FS_SPD')      -- wind failsafe speed (m/s)
assert(param:add_param(PARAM_TABLE_KEY, 3, 'WARN_IVL',    15), 'could not add WFS_WARN_IVL')    -- warning interval (s)
assert(param:add_param(PARAM_TABLE_KEY, 4, 'GPS_MGSPD',   40), 'could not add WFS_GPS_MGSPD')   -- GPS max ground speed (m/s), 0 to disable
assert(param:add_param(PARAM_TABLE_KEY, 5, 'GPS_MSAT',     6), 'could not add WFS_GPS_MSAT')    -- GPS min satellites, 0 to disable
assert(param:add_param(PARAM_TABLE_KEY, 6, 'GPS_MHDOP',   20), 'could not add WFS_GPS_MHDOP')   -- GPS max HDOP (m), 0 to disable
assert(param:add_param(PARAM_TABLE_KEY, 7, 'GPS_MSACC',   10), 'could not add WFS_GPS_MSACC')   -- GPS max speed accuracy (m/s), 0 to disable

local warn_speed        = Parameter('WFS_WARN_SPD')
local failsafe_speed    = Parameter('WFS_FS_SPD')
local warning_interval  = Parameter('WFS_WARN_IVL')
local gps_max_gnd_spd   = Parameter('WFS_GPS_MGSPD')
local gps_min_sats      = Parameter('WFS_GPS_MSAT')
local gps_max_hdop      = Parameter('WFS_GPS_MHDOP')
local gps_max_spd_acc   = Parameter('WFS_GPS_MSACC')

local warning_last_sent_ms = uint32_t()

local AHRS_WIND_MAX = Parameter('AHRS_WIND_MAX')
local AHRS_WIND_MAX_previous_value = AHRS_WIND_MAX:get()

local ARSPD_OPTIONS = Parameter('ARSPD_OPTIONS')
-- :get() returns a float; convert to integer for bitwise operations
local ARSPD_OPTIONS_previous_value = math.tointeger(ARSPD_OPTIONS:get()) or 0

function configure_for_bad_gps_data()
   AHRS_WIND_MAX_previous_value = AHRS_WIND_MAX:get()
   AHRS_WIND_MAX:set(0)  -- disable wind max check

   ARSPD_OPTIONS_previous_value = math.tointeger(ARSPD_OPTIONS:get()) or 0
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
   local ARSPD_OPTIONS_new_value = ARSPD_OPTIONS_previous_value | (1<<0)
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
   for i=0, gps:num_sensors()-1 do
      if gps_max_gnd_spd:get() > 0 and gps:ground_speed(i) > gps_max_gnd_spd:get() then
         return true
      end
      if gps_min_sats:get() > 0 and gps:num_sats(i) < gps_min_sats:get() then
         return true
      end
      if gps_max_hdop:get() > 0 and gps:horizontal_accuracy(i) > gps_max_hdop:get() then
         return true
      end
      if gps_max_spd_acc:get() > 0 and gps:speed_accuracy(i) > gps_max_spd_acc:get() then
         return true
      end
   end
   -- GPS data looks good
   return false
end

function update()

    -- adjust parameters and state if GPS data looks bad:
    reconfigure_based_on_gps_health()

    local wind = ahrs:get_wind() -- get the wind estimate
    if wind and not configured_for_bad_gps_data then
        -- make a 2D wind vector
        wind_xy = Vector2f()
        wind_xy:x(wind:x())
        wind_xy:y(wind:y())
        speed = wind_xy:length() -- compute the wind speed
        if speed > failsafe_speed:get() then
            gcs:send_text(0, "Wind failsafe at " .. speed .. " metres/second")
            vehicle:set_mode(11) -- FIXME: should be an enum.  11 is RTL.
            return
        end
        if speed > warn_speed:get() then
            if millis() - warning_last_sent_ms > uint32_t(warning_interval:get() * 1000) then
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
