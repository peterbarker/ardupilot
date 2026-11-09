--[[
  Locate this vehicle from a target whose global position is known.

  TARGET_ABSOLUTE gives the target's global position (e.g. from GNSS on a
  landing pad). TARGET_RELATIVE, from a sensor on this vehicle, gives the
  target's position relative to the vehicle. A TARGET_RELATIVE is paired with
  the latest TARGET_ABSOLUTE carrying the same id, and this vehicle's position
  (the target's position minus the relative position) is sent to the EKF as
  ExternalNav data through the visual odometry library.
--]]

local mavlink_msgs = require("MAVLink/mavlink_msgs")

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local MSGID_TARGET_ABSOLUTE = 510
local MSGID_TARGET_RELATIVE = 511
local msg_map = {
    [MSGID_TARGET_ABSOLUTE] = "TARGET_ABSOLUTE",
    [MSGID_TARGET_RELATIVE] = "TARGET_RELATIVE",
}
local MAV_FRAME = {LOCAL_NED=1, LOCAL_OFFSET_NED=7, BODY_FRD=12}
local SENSOR_CAPABILITY = {POSITION=1, VELOCITY=2}

local PARAM_TABLE_KEY = 98
local PARAM_TABLE_PREFIX = "TGTN_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), 'could not add param table')

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: TGTN_ENABLE
  // @DisplayName: Target ExternalNav enable
  // @Description: Enable sending this vehicle's position, derived from TARGET_ABSOLUTE and TARGET_RELATIVE, to the EKF as ExternalNav data. Requires VISO_TYPE=1 and an EKF origin
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local TGTN_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
  // @Param: TGTN_ABS_AGE
  // @DisplayName: Target ExternalNav TARGET_ABSOLUTE maximum age
  // @Description: A TARGET_RELATIVE is only used if the TARGET_ABSOLUTE with the same id arrived within this time of it. If TARGET_ABSOLUTE gives the target's velocity, the target's position is moved forward to the time of the TARGET_RELATIVE
  // @Units: s
  // @Range: 0.1 10
  // @User: Standard
--]]
local TGTN_ABS_AGE = bind_add_param("ABS_AGE", 2, 2)

mavlink:init(10, 2)
mavlink:register_rx_msgid(MSGID_TARGET_ABSOLUTE)
mavlink:register_rx_msgid(MSGID_TARGET_RELATIVE)

-- latest TARGET_ABSOLUTE for each id
local targets = {}
local warned_no_origin = false

local function is_finite(v)
    return v == v and v ~= math.huge and v ~= -math.huge
end

-- unknown (NaN) uncertainties contribute nothing; VISO_POS_M_NSE is the floor
local function sq_or_zero(v)
    if is_finite(v) then
        return v * v
    end
    return 0
end

local function handle_target_absolute(msg, time_ms)
    if (msg.sensor_capabilities & SENSOR_CAPABILITY.POSITION) == 0 or not is_finite(msg.alt) then
        return
    end
    local loc = Location()
    loc:lat(msg.lat)
    loc:lng(msg.lon)
    loc:alt(math.floor(msg.alt * 100 + 0.5))
    local vel = nil
    if (msg.sensor_capabilities & SENSOR_CAPABILITY.VELOCITY) ~= 0 then
        vel = Vector3f()
        vel:x(msg.vel[1])
        vel:y(msg.vel[2])
        vel:z(msg.vel[3])
    end
    targets[msg.id] = {
        time_ms = time_ms,
        loc = loc,
        vel = vel,
        pos_var = sq_or_zero(msg.position_std[1]),
    }
end

-- returns the target's position relative to the vehicle in NED, or nil
local function relative_position_NED(msg)
    local rel = Vector3f()
    rel:x(msg.x)
    rel:y(msg.y)
    rel:z(msg.z)
    local qs = msg.q_sensor
    if qs[1] ~= 0 or qs[2] ~= 0 or qs[3] ~= 0 or qs[4] ~= 0 then
        -- the position is in the sensor frame; q_sensor rotates it to vehicle-carried NED
        local q = Quaternion()
        q:q1(qs[1])
        q:q2(qs[2])
        q:q3(qs[3])
        q:q4(qs[4])
        q:normalize()
        -- despite its name, earth_to_body() rotates by q (sensor frame to NED here)
        q:earth_to_body(rel)
        return rel
    end
    if msg.frame == MAV_FRAME.LOCAL_NED or msg.frame == MAV_FRAME.LOCAL_OFFSET_NED then
        return rel
    end
    if msg.frame == MAV_FRAME.BODY_FRD then
        return ahrs:body_to_earth(rel)
    end
    return nil
end

local function handle_target_relative(msg, time_ms)
    local target = targets[msg.id]
    if target == nil then
        return
    end
    local age_s = (time_ms - target.time_ms) * 0.001
    if math.abs(age_s) > TGTN_ABS_AGE:get() then
        return
    end
    if not (is_finite(msg.x) and is_finite(msg.y) and is_finite(msg.z)) then
        return
    end
    local rel_ned = relative_position_NED(msg)
    if rel_ned == nil then
        return
    end
    local origin = ahrs:get_origin()
    if origin == nil then
        if not warned_no_origin then
            gcs:send_text(MAV_SEVERITY.WARNING, "target-extnav: no EKF origin")
            warned_no_origin = true
        end
        return
    end
    local attitude = ahrs:get_quaternion()
    if attitude == nil then
        return
    end

    -- target position relative to the EKF origin at the time of the TARGET_RELATIVE
    local target_ned = origin:get_distance_NED(target.loc)
    if target.vel ~= nil then
        target_ned = target_ned + target.vel:scale(age_s)
    end
    local pos = target_ned - rel_ned

    local rel_var = sq_or_zero(msg.position_std[1]) + sq_or_zero(msg.position_std[2]) + sq_or_zero(msg.position_std[3])
    local pos_err = math.sqrt(target.pos_var + rel_var)

    -- this gives no independent attitude, so the attitude error is set to its maximum
    visual_odom:handle_pose_estimate(time_ms, pos:x(), pos:y(), pos:z(), attitude, pos_err, 1.5, 0, 0)
end

local function update()
    if TGTN_ENABLE:get() <= 0 then
        return update, 1000
    end
    -- one message per run keeps each run within SCR_VM_I_COUNT; decoding checks the CRC in Lua
    local raw, _, receive_timestamp = mavlink:receive_chan()
    if raw == nil then
        return update, 20
    end
    local msg = mavlink_msgs.decode(raw, msg_map)
    if msg ~= nil then
        local time_ms = receive_timestamp:toint()
        if msg.msgid == MSGID_TARGET_ABSOLUTE then
            handle_target_absolute(msg, time_ms)
        elseif msg.msgid == MSGID_TARGET_RELATIVE then
            handle_target_relative(msg, time_ms)
        end
    end
    return update, 1
end

gcs:send_text(MAV_SEVERITY.INFO, "target-extnav script loaded")

return update()
