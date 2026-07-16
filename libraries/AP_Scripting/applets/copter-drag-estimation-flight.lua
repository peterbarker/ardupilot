--[[
 fly a Copter in a specific pattern to gather data for estimating
 bluff body and momentum drag, used for EKF3 wind estimation

 see https://youtu.be/xVVtvVuZGQE?t=1423

 How To Use
   - assign RC option 300 (Scripting1) to a spare transmitter switch
   - take off in LOITER and climb to somewhere in clean air, well away
     from obstacles, at least 20m above home.  Ensure there is
     downwind space for the vehicle to drift to the speed of the wind,
     and upwind space for four drive-and-drift cycles
   - change to mode GUIDED
   - move the switch high
   - be ready to take control again in LOITER if required
   - wait for the script to report that it is done

 The vehicle first holds a level attitude and drifts until it is
 moving with the air mass; the terminal drift velocity is the wind
 estimate.  It then flies four runs, one for each vehicle axis
 orientation relative to the wind (nose-in, right-side-in, tail-in,
 left-side-in).  Each run repositions to the drift end point, yaws to
 the test heading, drives into the wind at a fixed lean angle until it
 stops accelerating, then levels and drifts back downwind.  Finally
 the vehicle returns to the position the initial drift started from.
--]]

local UPDATE_INTERVAL_MS = 10    -- script updates at 100Hz
local AUX_FUNCTION = 300         -- Scripting1 auxiliary switch function
local MODE_GUIDED = 4

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local DRIVE_LEAN_ANGLE_DEG = 30  -- lean angle when driving into wind
local ACCEL_SETTLED_MAX = 0.15   -- filtered horizontal accel (m/s/s) below which we are at terminal velocity
local ACCEL_LPF_HZ = 0.5         -- low-pass filter corner for settle detection
local WIND_SPEED_MIN = 1.0       -- minimum windspeed (m/s) for the test to be useful
local MIN_ALT_M = 20             -- minimum altitude above home for the script to run
local INIT_DRIFT_MIN_MS = 8000   -- minimum time to drift for the wind measurement
local SETTLE_HOLD_MS = 2000      -- time accel must stay low to be considered settled
local DRIVE_MIN_MS = 5000        -- minimum time to drive into wind
local DRIVE_MAX_MS = 60000       -- give up if still accelerating after this long
local DRIFT_MIN_MS = 5000        -- minimum time to drift back with the wind
local DRIFT_MAX_MS = 90000       -- give up if drift has not settled after this long
local YAW_ERR_MAX_DEG = 3        -- heading acceptance for the yaw phase
local YAW_HOLD_MS = 1000         -- time heading must be held to be considered achieved
local DATUM_RADIUS_M = 4         -- reposition acceptance radius
local RETURN_RADIUS_M = 2        -- final return acceptance radius
local MAX_DIST_M = 800           -- abort if further than this from the start position

local RUN_OFFSETS_DEG = { 0, 90, 180, 270 }  -- vehicle yaw relative to wind-from heading

-- test state; nil when no test is running
local run
local abort_reason
local state = "unknown"
local last_state_emit_ms = uint32_t(0)

local function progress(severity, text)
    gcs:send_text(severity, string.format("DRGE: %s", text))
end

local function wrap_180(angle_deg)
    local a = angle_deg % 360
    if a > 180 then
        a = a - 360
    end
    return a
end

local function wrap_360(angle_deg)
    return angle_deg % 360
end

local function abort_test(reason)
    abort_reason = reason
    progress(MAV_SEVERITY.WARNING, string.format("aborted: %s", reason))
    run = nil
end

-- one-pole low-pass filter; initialises on first sample
local function lpf_apply(filter, sample, dt, cutoff_hz)
    if filter.value == nil then
        filter.value = sample
    else
        local alpha = dt / (dt + 1.0/(2.0 * math.pi * cutoff_hz))
        filter.value = filter.value + (sample - filter.value) * alpha
    end
    return filter.value
end

-- magnitude of earth-frame horizontal acceleration.  gravity is
-- vertical in the earth frame so this is zero at constant velocity
-- regardless of vehicle attitude
local function horizontal_accel()
    local accel_ef = ahrs:body_to_earth(ahrs:get_accel())
    local x = accel_ef:x()
    local y = accel_ef:y()
    return math.sqrt(x*x + y*y)
end

-- watch for the vehicle reaching terminal velocity in level flight
-- and average the groundspeed once it has; returns the averaged
-- groundspeed north,east components once settled, nil until then
local function settled_groundspeed(now, min_elapsed_ms)
    if now - run.phase_start_ms < min_elapsed_ms or
        run.accel_filter.value == nil or
        run.accel_filter.value > ACCEL_SETTLED_MAX then
        run.settle_start_ms = nil
        return nil
    end
    if run.settle_start_ms == nil then
        run.settle_start_ms = now
        run.gs_sum_n = 0.0
        run.gs_sum_e = 0.0
        run.gs_count = 0
    end
    local gs = ahrs:groundspeed_vector()
    run.gs_sum_n = run.gs_sum_n + gs:x()
    run.gs_sum_e = run.gs_sum_e + gs:y()
    run.gs_count = run.gs_count + 1
    if now - run.settle_start_ms < SETTLE_HOLD_MS then
        return nil
    end
    return run.gs_sum_n / run.gs_count, run.gs_sum_e / run.gs_count
end

local function start_phase(phase, now)
    run.phase = phase
    run.phase_start_ms = now
    run.settle_start_ms = nil
    run.yaw_hold_start_ms = nil
end

-- yaw of the vehicle for the current run
local function run_target_yaw_deg()
    return wrap_360(run.wind_from_deg + RUN_OFFSETS_DEG[run.run_index])
end

local function send_level_attitude(yaw_deg)
    vehicle:set_target_angle_and_climbrate(0, 0, yaw_deg, 0, false, 0)
end

-- returns distance in metres to pos (an origin-relative Vector3f)
local function distance_to(pos)
    local here = ahrs:get_relative_position_NED_origin()
    if here == nil then
        return nil
    end
    return (pos - here):length()
end

local function handle_init_drift(now)
    send_level_attitude(run.hold_yaw_deg)
    state = string.format("initial drift accel=%.2f", run.accel_filter.value or 0)
    local wind_n, wind_e = settled_groundspeed(now, INIT_DRIFT_MIN_MS)
    if wind_n == nil or wind_e == nil then
        return
    end
    local windspeed = math.sqrt(wind_n*wind_n + wind_e*wind_e)
    if windspeed < WIND_SPEED_MIN then
        abort_test(string.format("wind %.1fm/s below %.1fm/s min", windspeed, WIND_SPEED_MIN))
        return
    end
    run.wind_n = wind_n
    run.wind_e = wind_e
    run.wind_from_deg = wrap_360(math.deg(math.atan(wind_e, wind_n)) + 180)
    progress(MAV_SEVERITY.NOTICE, string.format("wind %.1fm/s from %.0fdeg", windspeed, run.wind_from_deg))
    run.datum_pos = ahrs:get_relative_position_NED_origin()
    run.run_index = 1
    start_phase("move_to_datum", now)
end

local function handle_move_to_datum(now)
    vehicle:set_target_posvel_NED(run.datum_pos, Vector3f())
    local dist = distance_to(run.datum_pos)
    if dist == nil then
        return
    end
    state = string.format("run %u: to datum dist=%.1f", run.run_index, dist)
    if dist < DATUM_RADIUS_M then
        start_phase("yaw", now)
    end
end

local function handle_yaw(now)
    local target_yaw = run_target_yaw_deg()
    send_level_attitude(target_yaw)
    local yaw_err = wrap_180(target_yaw - math.deg(ahrs:get_yaw_rad()))
    state = string.format("run %u: yaw want=%.0f err=%.1f", run.run_index, target_yaw, yaw_err)
    if math.abs(yaw_err) > YAW_ERR_MAX_DEG then
        run.yaw_hold_start_ms = nil
        return
    end
    if run.yaw_hold_start_ms == nil then
        run.yaw_hold_start_ms = now
        return
    end
    if now - run.yaw_hold_start_ms > YAW_HOLD_MS then
        start_phase("drive", now)
    end
end

local function handle_drive(now)
    -- lean towards the wind-from direction while holding the run heading
    local offset_rad = math.rad(RUN_OFFSETS_DEG[run.run_index])
    local roll = -DRIVE_LEAN_ANGLE_DEG * math.sin(offset_rad)
    local pitch = -DRIVE_LEAN_ANGLE_DEG * math.cos(offset_rad)
    vehicle:set_target_angle_and_climbrate(roll, pitch, run_target_yaw_deg(), 0, false, 0)

    local elapsed = now - run.phase_start_ms
    state = string.format("run %u: drive accel=%.2f", run.run_index, run.accel_filter.value or 0)
    if elapsed > DRIVE_MAX_MS then
        abort_test(string.format("run %u: drive did not settle", run.run_index))
        return
    end
    if elapsed > DRIVE_MIN_MS and run.accel_filter.value ~= nil and run.accel_filter.value < ACCEL_SETTLED_MAX then
        start_phase("drift", now)
    end
end

local function handle_drift(now)
    send_level_attitude(run_target_yaw_deg())
    state = string.format("run %u: drift accel=%.2f", run.run_index, run.accel_filter.value or 0)
    if now - run.phase_start_ms > DRIFT_MAX_MS then
        abort_test(string.format("run %u: drift did not settle", run.run_index))
        return
    end
    local wind_n = settled_groundspeed(now, DRIFT_MIN_MS)
    if wind_n == nil then
        return
    end
    if run.run_index >= #RUN_OFFSETS_DEG then
        start_phase("return_to_start", now)
        return
    end
    run.run_index = run.run_index + 1
    start_phase("move_to_datum", now)
end

local function handle_return_to_start(now)
    vehicle:set_target_posvel_NED(run.start_pos, Vector3f())
    local dist = distance_to(run.start_pos)
    if dist == nil then
        return
    end
    state = string.format("return dist=%.1f", dist)
    if dist < RETURN_RADIUS_M then
        progress(MAV_SEVERITY.NOTICE, "done")
        start_phase("done", now)
    end
end

local function handle_done()
    send_level_attitude(run.hold_yaw_deg)
    state = "done"
end

local phase_handlers = {
    init_drift = handle_init_drift,
    move_to_datum = handle_move_to_datum,
    yaw = handle_yaw,
    drive = handle_drive,
    drift = handle_drift,
    return_to_start = handle_return_to_start,
    done = handle_done,
}

local function new_test(now)
    progress(MAV_SEVERITY.NOTICE, "starting")
    return {
        phase = "init_drift",
        phase_start_ms = now,
        start_pos = ahrs:get_relative_position_NED_origin(),
        hold_yaw_deg = math.deg(ahrs:get_yaw_rad()),
        accel_filter = {},
        last_update_ms = now,
        run_index = 0,
    }
end

function update()
    local now = millis()

    -- if no aux switch allocation then do nothing
    local aux_switch = rc:find_channel_for_option(AUX_FUNCTION)
    if aux_switch == nil then
        run = nil
        state = string.format("want RCx_OPTION=%u", AUX_FUNCTION)
    elseif aux_switch:get_aux_switch_pos() ~= 2 then
        -- lowering the switch stops the test and clears any abort
        run = nil
        abort_reason = nil
        state = "want switch high"
    elseif abort_reason ~= nil then
        -- hold in aborted state until the switch is cycled
        state = string.format("aborted: %s", abort_reason)
    else
        local relpos_home = ahrs:get_relative_position_NED_home()
        local relpos_origin = ahrs:get_relative_position_NED_origin()
        if relpos_home == nil or relpos_origin == nil then
            if run ~= nil then
                abort_test("lost position estimate")
            else
                state = "want position estimate"
            end
        elseif -relpos_home:z() < MIN_ALT_M then
            if run ~= nil then
                abort_test(string.format("below %um", MIN_ALT_M))
            else
                state = string.format("want altitude >%um", MIN_ALT_M)
            end
        elseif vehicle:get_mode() ~= MODE_GUIDED then
            if run ~= nil then
                abort_test("not in GUIDED")
            else
                state = "want mode GUIDED"
            end
        else
            if run == nil then
                run = new_test(now)
            end

            -- track filtered horizontal acceleration for settle detection
            local dt = (now - run.last_update_ms):tofloat() * 0.001
            run.last_update_ms = now
            if dt > 0 then
                lpf_apply(run.accel_filter, horizontal_accel(), dt, ACCEL_LPF_HZ)
            end

            if run.phase ~= "done" then
                local dist = distance_to(run.start_pos)
                if dist ~= nil and dist > MAX_DIST_M then
                    abort_test(string.format("%.0fm from start", dist))
                end
            end

            if run ~= nil then
                phase_handlers[run.phase](now)
            end
        end
    end

    if now - last_state_emit_ms > 1000 then
        progress(MAV_SEVERITY.INFO, state)
        last_state_emit_ms = now
    end

    return update, UPDATE_INTERVAL_MS
end

return update()
