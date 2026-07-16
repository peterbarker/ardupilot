--[[
 fly a Copter in a specific pattern to measure bluff body and
 momentum drag, computing and saving the EK3_DRAG_BCOEF_X,
 EK3_DRAG_BCOEF_Y and EK3_DRAG_MCOEF parameters used for EKF3 wind
 estimation

 WARNING: on successful completion this script writes the
 EK3_DRAG_BCOEF_X, EK3_DRAG_BCOEF_Y and EK3_DRAG_MCOEF parameters,
 changing EKF3 wind and airspeed estimation behaviour

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

 For a multicopter thrust is along the body Z axis, so the body-frame
 X and Y specific forces measured by the accelerometers are purely
 aerodynamic drag, at any attitude.  During each level drift the
 vehicle decelerates from its maximum airspeed through zero and back
 up to the wind speed, and the script records the filtered body-frame
 airspeed and specific force.  Those samples are fitted, per axis, to
 the EKF3 drag model

    accel = -sign(V)*(0.5*rho/BCOEF)*V^2 - MCOEF*(rho/1.225)*V

 by least squares (with an intercept absorbing accelerometer bias),
 yielding the ballistic coefficient for each axis and the shared
 momentum drag coefficient.  The wind estimate is refreshed at the
 end of every drift, when the vehicle is again moving with the air.
--]]

local UPDATE_INTERVAL_MS = 10    -- script updates at 100Hz
local AUX_FUNCTION = 300         -- Scripting1 auxiliary switch function
local MODE_GUIDED = 4

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local ACCEL_LPF_HZ = 0.5         -- low-pass filter corner for settle detection
local SAMPLE_LPF_HZ = 2.0        -- low-pass filter corner for the drag regression signals
local SAMPLE_SETTLE_MS = 1500    -- delay after levelling before drift samples are taken
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

local RHO_SSL = 1.225            -- sea-level standard air density, kg/m^3
local AIR_GAS_CONSTANT = 287.05  -- specific gas constant for dry air, J/kg/K
local C_TO_KELVIN = 273.15

local WIND_ACC_FACTOR = 1/3      -- fraction of DRGE_ACC_MAX the wind measurement settles to

local FIT_MIN_SAMPLES = 200      -- minimum regression samples per axis
local FIT_MIN_SIGN_SAMPLES = 50  -- minimum samples with each airspeed sign per axis
local BCOEF_SAVE_MIN = 1.0       -- EKF3 treats BCOEF <= 1 as disabled
local BCOEF_SAVE_MAX = 150.0
local MCOEF_SAVE_MIN = 0.0
local MCOEF_SAVE_MAX = 1.0

local RUN_OFFSETS_DEG = { 0, 90, 180, 270 }  -- vehicle yaw relative to wind-from heading

local PARAM_TABLE_KEY = 97
local PARAM_TABLE_PREFIX = "DRGE_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), 'could not add param table')

--[[
  // @Param: DRGE_ANGLE
  // @DisplayName: drag estimation drive lean angle
  // @Description: Lean angle used when driving the vehicle into the wind to build up airspeed
  // @Range: 10 45
  // @Units: deg
  // @User: Standard
--]]
local DRGE_ANGLE = bind_add_param('ANGLE', 1, 30)

--[[
  // @Param: DRGE_SPD_MIN
  // @DisplayName: drag estimation minimum sample airspeed
  // @Description: Minimum body-axis relative airspeed for a sample to contribute to the drag fit. Samples slower than this carry little drag information
  // @Range: 0.5 5
  // @Units: m/s
  // @User: Standard
--]]
local DRGE_SPD_MIN = bind_add_param('SPD_MIN', 2, 2)

--[[
  // @Param: DRGE_ACC_MAX
  // @DisplayName: drag estimation settled acceleration
  // @Description: Filtered horizontal acceleration below which the vehicle is considered to have stopped accelerating at the end of the drive into wind. The wind measurement at terminal drift velocity settles to one third of this; residual acceleration there means residual airspeed, which becomes a systematic error in the drag fit
  // @Range: 0.05 0.5
  // @Units: m/s/s
  // @User: Standard
--]]
local DRGE_ACC_MAX = bind_add_param('ACC_MAX', 3, 0.15)

--[[
  // @Param: DRGE_DIST_MAX
  // @DisplayName: drag estimation maximum distance
  // @Description: The test is aborted if the vehicle moves further than this from the position at which the test was started
  // @Range: 50 2000
  // @Units: m
  // @User: Standard
--]]
local DRGE_DIST_MAX = bind_add_param('DIST_MAX', 4, 800)

--[[
  // @Param: DRGE_WND_MIN
  // @DisplayName: drag estimation minimum wind speed
  // @Description: Minimum wind speed for the test to proceed. The test is aborted if the initial terminal drift velocity, or any later refresh of the wind measurement, is below this
  // @Range: 0.5 10
  // @Units: m/s
  // @User: Standard
--]]
local DRGE_WND_MIN = bind_add_param('WND_MIN', 5, 1)

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
-- groundspeed north,east components once settled, nil until then.
-- The threshold must be tight: the residual airspeed when it is met
-- becomes a systematic error in the wind measurement and hence in
-- every airspeed used by the drag fit
local function settled_groundspeed(now, min_elapsed_ms)
    if now - run.phase_start_ms < min_elapsed_ms or
        run.accel_filter.value == nil or
        run.accel_filter.value > DRGE_ACC_MAX:get() * WIND_ACC_FACTOR then
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
    if phase == "drift" then
        -- restart the regression signal filters; the level-off
        -- transient must not bleed into the samples
        run.sample_filters = { ax = {}, ay = {}, vx = {}, vy = {} }
    end
end

-- accumulate one regression sample for one body axis.  V is the
-- body-frame relative airspeed component, a the body-frame specific
-- force component.  The EKF3 drag model being fitted is
--    a = -sign(V)*(0.5*rho/BCOEF)*V^2 - MCOEF*(rho/1.225)*V
-- which is linear in the regressors
--    u = -0.5*rho*V*|V|        with coefficient A = 1/BCOEF
--    w = -(rho/1.225)*V        with coefficient B = MCOEF
-- plus an intercept absorbing accelerometer bias
local function accumulate_sample(axis, V, a)
    if math.abs(V) < DRGE_SPD_MIN:get() then
        return
    end
    local u = -0.5 * run.rho * V * math.abs(V)
    local w = -(run.rho / RHO_SSL) * V
    axis.n = axis.n + 1
    axis.Su = axis.Su + u
    axis.Sw = axis.Sw + w
    axis.Sa = axis.Sa + a
    axis.Suu = axis.Suu + u*u
    axis.Suw = axis.Suw + u*w
    axis.Sua = axis.Sua + u*a
    axis.Sww = axis.Sww + w*w
    axis.Swa = axis.Swa + w*a
    if V > 0 then
        axis.n_pos = axis.n_pos + 1
    else
        axis.n_neg = axis.n_neg + 1
    end
end

-- sample body-frame relative airspeed and specific force for the
-- drag regression.  All four signals pass through identical filters
-- so the regression pairs stay phase-matched
local function sample_drag(dt)
    local vel = ahrs:get_velocity_NED()
    if vel == nil then
        return
    end
    local rel_wind_ef = Vector3f()
    rel_wind_ef:x(vel:x() - run.wind_n)
    rel_wind_ef:y(vel:y() - run.wind_e)
    rel_wind_ef:z(vel:z())
    local rel_wind_bf = ahrs:earth_to_body(rel_wind_ef)
    local accel = ahrs:get_accel()
    local f = run.sample_filters
    local vx = lpf_apply(f.vx, rel_wind_bf:x(), dt, SAMPLE_LPF_HZ)
    local vy = lpf_apply(f.vy, rel_wind_bf:y(), dt, SAMPLE_LPF_HZ)
    local ax = lpf_apply(f.ax, accel:x(), dt, SAMPLE_LPF_HZ)
    local ay = lpf_apply(f.ay, accel:y(), dt, SAMPLE_LPF_HZ)
    accumulate_sample(run.fit_x, vx, ax)
    accumulate_sample(run.fit_y, vy, ay)
end

-- solve the per-axis normal equations for [A B C] where
-- a = A*u + B*w + C.  Returns bcoef, mcoef, bias or nil and a reason
local function fit_axis(axis)
    if axis.n < FIT_MIN_SAMPLES or
        axis.n_pos < FIT_MIN_SIGN_SAMPLES or
        axis.n_neg < FIT_MIN_SIGN_SAMPLES then
        return nil, string.format("too few samples %u +%u -%u", axis.n, axis.n_pos, axis.n_neg)
    end
    local n = axis.n
    local det = axis.Suu*(axis.Sww*n - axis.Sw*axis.Sw)
        - axis.Suw*(axis.Suw*n - axis.Sw*axis.Su)
        + axis.Su*(axis.Suw*axis.Sw - axis.Sww*axis.Su)
    if math.abs(det) <= 1.0e-9 * math.max(axis.Suu, 1.0) * math.max(axis.Sww, 1.0) * n then
        return nil, "singular fit"
    end
    local A = (axis.Sua*(axis.Sww*n - axis.Sw*axis.Sw)
        - axis.Suw*(axis.Swa*n - axis.Sw*axis.Sa)
        + axis.Su*(axis.Swa*axis.Sw - axis.Sww*axis.Sa)) / det
    local B = (axis.Suu*(axis.Swa*n - axis.Sw*axis.Sa)
        - axis.Sua*(axis.Suw*n - axis.Sw*axis.Su)
        + axis.Su*(axis.Suw*axis.Sa - axis.Swa*axis.Su)) / det
    local C = (axis.Suu*(axis.Sww*axis.Sa - axis.Swa*axis.Sw)
        - axis.Suw*(axis.Suw*axis.Sa - axis.Swa*axis.Su)
        + axis.Sua*(axis.Suw*axis.Sw - axis.Sww*axis.Su)) / det
    if A <= 0 then
        return nil, string.format("bluff term %.4f not positive", A)
    end
    return 1.0/A, B, C
end

-- yaw of the vehicle for the current run
local function run_target_yaw_deg()
    return wrap_360(run.wind_from_deg + RUN_OFFSETS_DEG[run.run_index])
end

local function send_level_attitude(yaw_deg)
    vehicle:set_target_angle_and_climbrate(0, 0, yaw_deg, 0, false, 0)
end

-- record a wind measurement taken at terminal drift velocity,
-- aborting the test if the wind is too light to be useful
local function set_wind(wind_n, wind_e)
    local windspeed = math.sqrt(wind_n*wind_n + wind_e*wind_e)
    if windspeed < DRGE_WND_MIN:get() then
        abort_test(string.format("wind %.1fm/s below %.1fm/s min", windspeed, DRGE_WND_MIN:get()))
        return false
    end
    run.wind_n = wind_n
    run.wind_e = wind_e
    run.wind_from_deg = wrap_360(math.deg(math.atan(wind_e, wind_n)) + 180)
    progress(MAV_SEVERITY.NOTICE, string.format("wind %.1fm/s from %.0fdeg", windspeed, run.wind_from_deg))
    return true
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
    if not set_wind(wind_n, wind_e) then
        return
    end
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
    local roll = -DRGE_ANGLE:get() * math.sin(offset_rad)
    local pitch = -DRGE_ANGLE:get() * math.cos(offset_rad)
    vehicle:set_target_angle_and_climbrate(roll, pitch, run_target_yaw_deg(), 0, false, 0)

    local elapsed = now - run.phase_start_ms
    state = string.format("run %u: drive accel=%.2f", run.run_index, run.accel_filter.value or 0)
    if elapsed > DRIVE_MAX_MS then
        abort_test(string.format("run %u: drive did not settle", run.run_index))
        return
    end
    if elapsed > DRIVE_MIN_MS and run.accel_filter.value ~= nil and run.accel_filter.value < DRGE_ACC_MAX:get() then
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
    if now - run.phase_start_ms > SAMPLE_SETTLE_MS then
        sample_drag(run.dt)
    end
    local wind_n, wind_e = settled_groundspeed(now, DRIFT_MIN_MS)
    if wind_n == nil or wind_e == nil then
        return
    end
    -- back at terminal drift velocity: refresh the wind measurement
    if not set_wind(wind_n, wind_e) then
        return
    end
    if run.run_index >= #RUN_OFFSETS_DEG then
        start_phase("fit", now)
        return
    end
    run.run_index = run.run_index + 1
    start_phase("move_to_datum", now)
end

-- fit the accumulated samples, report the results and save the EK3
-- drag parameters
local function handle_fit(now)
    local bcoef_x, mcoef_x, bias_x = fit_axis(run.fit_x)
    local bcoef_y, mcoef_y, bias_y = fit_axis(run.fit_y)
    if bcoef_x == nil then
        progress(MAV_SEVERITY.ERROR, string.format("X fit failed: %s", mcoef_x))
    end
    if bcoef_y == nil then
        progress(MAV_SEVERITY.ERROR, string.format("Y fit failed: %s", mcoef_y))
    end
    if bcoef_x ~= nil and bcoef_y ~= nil then
        local mcoef = 0.5 * (mcoef_x + mcoef_y)
        progress(MAV_SEVERITY.NOTICE, string.format("BCOEF_X=%.2f n=%u", bcoef_x, run.fit_x.n))
        progress(MAV_SEVERITY.NOTICE, string.format("BCOEF_Y=%.2f n=%u", bcoef_y, run.fit_y.n))
        progress(MAV_SEVERITY.NOTICE, string.format("MCOEF=%.3f", mcoef))
        progress(MAV_SEVERITY.INFO, string.format("mcoef x=%.3f y=%.3f", mcoef_x, mcoef_y))
        progress(MAV_SEVERITY.INFO, string.format("bias x=%.3f y=%.3f rho=%.3f", bias_x, bias_y, run.rho))
        if bcoef_x < BCOEF_SAVE_MIN or bcoef_x > BCOEF_SAVE_MAX or
            bcoef_y < BCOEF_SAVE_MIN or bcoef_y > BCOEF_SAVE_MAX or
            mcoef < MCOEF_SAVE_MIN or mcoef > MCOEF_SAVE_MAX then
            progress(MAV_SEVERITY.ERROR, "results out of range, not saved")
        else
            param:set_and_save('EK3_DRAG_BCOEF_X', bcoef_x)
            param:set_and_save('EK3_DRAG_BCOEF_Y', bcoef_y)
            param:set_and_save('EK3_DRAG_MCOEF', mcoef)
            progress(MAV_SEVERITY.NOTICE, "params saved")
        end
    end
    start_phase("return_to_start", now)
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
    fit = handle_fit,
    return_to_start = handle_return_to_start,
    done = handle_done,
}

local function new_fit_accumulator()
    return {
        n = 0, n_pos = 0, n_neg = 0,
        Su = 0.0, Sw = 0.0, Sa = 0.0,
        Suu = 0.0, Suw = 0.0, Sua = 0.0,
        Sww = 0.0, Swa = 0.0,
    }
end

local function new_test(now)
    progress(MAV_SEVERITY.NOTICE, "starting")
    return {
        phase = "init_drift",
        phase_start_ms = now,
        start_pos = ahrs:get_relative_position_NED_origin(),
        hold_yaw_deg = math.deg(ahrs:get_yaw_rad()),
        accel_filter = {},
        last_update_ms = now,
        dt = 0.0,
        run_index = 0,
        rho = baro:get_pressure() / (AIR_GAS_CONSTANT * (baro:get_temperature() + C_TO_KELVIN)),
        fit_x = new_fit_accumulator(),
        fit_y = new_fit_accumulator(),
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
            run.dt = dt
            if dt > 0 then
                lpf_apply(run.accel_filter, horizontal_accel(), dt, ACCEL_LPF_HZ)
            end

            if run.phase ~= "done" then
                local dist = distance_to(run.start_pos)
                if dist ~= nil and dist > DRGE_DIST_MAX:get() then
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
