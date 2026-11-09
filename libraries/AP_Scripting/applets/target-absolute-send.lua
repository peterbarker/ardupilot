-- Send the TARGET_ABSOLUTE mavlink message to allow other vehicles to follow this one
--
-- How To Use
-- 1. copy this script to the autopilot's "scripts" directory
-- 2. within the "scripts" directory create a "modules" directory
-- 3. copy the MAVLink/mavlink_msgs_xxx files to the "scripts" directory
-- 4. the TARGET_ABSOLUTE message will be published at 10hz

-- load mavlink message definitions from modules/MAVLink directory
local mavlink_msgs = require("MAVLink/mavlink_msgs")

-- global definitions
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local UPDATE_INTERVAL_MS = 100                  -- update at about 10hz
local SENSOR_CAPABILITY = {POSITION=1, VELOCITY=2, ACCELERATION=4, ATTITUDE=8, RATES=16}
local NaN = 0/0

 -- setup script specific parameters
local PARAM_TABLE_KEY = 97
local PARAM_TABLE_PREFIX = "TGTA_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), 'could not add param table')

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: TGTA_ENABLE
  // @DisplayName: Target Absolute Send Enable
  // @Description: Target Absolute Send Enable
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local TGTA_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
  // @Param: TGTA_MAV_CHAN
  // @DisplayName: Target Absolute Send MAVLink Channel
  // @Description: MAVLink channel to which TARGET_ABSOLUTE should be sent
  // @Range: 0 10
  // @User: Standard
--]]
local TGTA_MAV_CHAN = bind_add_param("MAV_CHAN", 2, 0)

-- send TARGET_ABSOLUTE message
local function send_target_absolute_msg()

    local curr_loc = ahrs:get_location()
    if curr_loc == nil then
        return
    end
    local capabilities = SENSOR_CAPABILITY.POSITION

    -- velocity in m/s in NED frame
    local vel = {0, 0, 0}
    local vel_NED = ahrs:get_velocity_NED()
    if vel_NED ~= nil then
        capabilities = capabilities | SENSOR_CAPABILITY.VELOCITY
        vel = {vel_NED:x(), vel_NED:y(), vel_NED:z()}
    end

    -- attitude from body FRD to NED
    local q_target = {1, 0, 0, 0}
    local attitude_quat = ahrs:get_quaternion()
    if attitude_quat ~= nil then
        capabilities = capabilities | SENSOR_CAPABILITY.ATTITUDE
        q_target = {attitude_quat:q1(), attitude_quat:q2(), attitude_quat:q3(), attitude_quat:q4()}
    end

    -- body-frame rates in rad/s
    local gyro = ahrs:get_gyro()
    capabilities = capabilities | SENSOR_CAPABILITY.RATES

    local msg = {}
    -- micros() wraps after 71 minutes, millis() after 49 days
    msg.time_usec = millis():toint() * 1000
    msg.id = 0
    msg.sensor_capabilities = capabilities
    msg.lat = curr_loc:lat()
    msg.lon = curr_loc:lng()
    msg.alt = curr_loc:alt() * 0.01
    msg.vel = vel
    msg.acc = {0, 0, 0}
    msg.q_target = q_target
    msg.rates = {gyro:x(), gyro:y(), gyro:z()}
    msg.position_std = {NaN, NaN}
    msg.vel_std = {NaN, NaN, NaN}
    msg.acc_std = {NaN, NaN, NaN}

    mavlink:send_chan(TGTA_MAV_CHAN:get(), mavlink_msgs.encode("TARGET_ABSOLUTE", msg))
end

gcs:send_text(MAV_SEVERITY.INFO, "target-absolute-send script loaded")

local function update()

    -- exit immediately if not enabled
    if (TGTA_ENABLE:get() <= 0) then
        return update, 1000
    end

    send_target_absolute_msg()

    return update, UPDATE_INTERVAL_MS
end

return update()
