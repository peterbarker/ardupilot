/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_MotorsHeli_Quad_DDVP.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Quad_DDVP::var_info[] = {
    // nest AP_MotorsHeli directly rather than via AP_MotorsHeli_Quad,
    // whose table holds nothing but the same nested hop: group ids
    // only fit three levels of nesting and the heli tree already uses
    // two below this table (indices 1-3 remain reserved to match)
    AP_NESTEDGROUPINFO(AP_MotorsHeli, 0),

    // the nested group shares one index space with AP_MotorsHeli;
    // indexes 50-63 are reserved for this class

    // @Param: DRSC_TC
    // @DisplayName: Dynamic rotor speed demand time constant
    // @Description: Time constant for changes in the per-rotor speed demand. Larger values move the macro thrust changes more slowly onto the drive motors, leaving more work for blade pitch.
    // @Range: 0.1 10
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("DRSC_TC", 50, AP_MotorsHeli_Quad_DDVP, _speed_tc, 2.0f),

    // @Param: DRSC_RTC
    // @DisplayName: Dynamic rotor speed response time constant
    // @Description: Modelled time constant of the rotor speed response to a demand change, used to feedforward blade pitch while the rotor is still spooling. Should match the real rotor and drive's response.
    // @Range: 0.1 10
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("DRSC_RTC", 51, AP_MotorsHeli_Quad_DDVP, _rotor_tc, 1.5f),

    // @Param: DRSC_COL_TRIM
    // @DisplayName: Dynamic rotor speed collective trim point
    // @Description: Blade pitch the dynamic rotor speed allocation re-centres to in the steady state, as a fraction of full collective travel. Lower values keep more pitch authority in reserve but demand higher rotor speeds.
    // @Range: 0.1 0.9
    // @User: Advanced
    AP_GROUPINFO("DRSC_COL_TRIM", 52, AP_MotorsHeli_Quad_DDVP, _coll_trim, 0.4f),

    // @Param: DRSC_SPD_MIN
    // @DisplayName: Dynamic rotor speed minimum
    // @Description: Minimum per-rotor speed demand as a fraction of full speed once the rotors are running.
    // @Range: 0.1 1.0
    // @User: Advanced
    AP_GROUPINFO("DRSC_SPD_MIN", 53, AP_MotorsHeli_Quad_DDVP, _speed_min, 0.3f),

    // @Param: DRSC_RPM_MAX
    // @DisplayName: Dynamic rotor speed full-speed rotor RPM
    // @Description: Rotor speed at full drive motor output, used to normalise measured rotor speed from drive motor telemetry for blade pitch feedforward and drive failure detection.
    // @Range: 100 10000
    // @Units: RPM
    // @User: Advanced
    AP_GROUPINFO("DRSC_RPM_MAX", 54, AP_MotorsHeli_Quad_DDVP, _rpm_max, 1500),

    // @Param: DRSC_SPD_FW
    // @DisplayName: Fixed-wing minimum rotor speed
    // @Description: Minimum per-rotor speed demand while the rotor is providing forward thrust in fixed-wing flight. Raising this above DRSC_SPD_MIN acts as a cruise governor: blade pitch flattens to hold the demanded thrust at the higher speed, keeping pitch authority (thrust scales with speed squared) in reserve for gusts and the back-transition, at the cost of profile drag. Zero uses DRSC_SPD_MIN.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("DRSC_SPD_FW", 55, AP_MotorsHeli_Quad_DDVP, _speed_min_fw, 0),

    AP_GROUPEND
};

// init_outputs
void AP_MotorsHeli_Quad_DDVP::init_outputs()
{
    if (initialised_ok()) {
        return;
    }

    AP_MotorsHeli_Quad::init_outputs();

    // the per-rotor drive motors follow the collective servos
    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        add_motor_num(CH_5+i);
    }

    set_initialised_ok(_frame_class == MOTOR_FRAME_HELI_QUAD_DDVP);
}

// move_actuators - moves swash plate to attitude of parameters passed in
void AP_MotorsHeli_Quad_DDVP::move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out)
{
    AP_MotorsHeli_Quad::move_actuators(roll_out, pitch_out, coll_in, yaw_out);

    // With the rotors stopped there is no rotor speed to reallocate the
    // thrust demand against, and the reallocation below (which assumes
    // spinning rotors) would pin the blade pitch at the zero-thrust
    // point. Leave the base collective in place so the blade pitch
    // tracks the commanded collective and the head can be exercised on
    // the ground for setup (H_SV_MAN, H_SV_TEST). The drive motors are
    // held at zero by esc_output() while disarmed regardless, so only
    // the blade-pitch servos are affected.
    if (!armed()) {
        for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
            _speed_demand[i] = _speed_min;
            _rotor_speed_model[i] = 0.0f;
            _rotor_speed_est[i] = 0.0f;
        }
        return;
    }

    // _out[] now holds per-rotor blade pitch in -1..1 about the
    // zero-thrust point, computed assuming full rotor speed.
    // Reallocate between the two actuators: the slow per-rotor speed
    // demand tracks the thrust required so that blade pitch
    // re-centres to +/- _coll_trim in the steady state, and blade
    // pitch supplies the fast residual, feeding forward against the
    // modelled rotor speed
    const float zero_out = _collective_zero_thrust_pct * 2.0f - 1.0f;

    // signed thrust demands in full-rotor-speed blade pitch units
    float thrusts[AP_MOTORS_HELI_QUAD_NUM_MOTORS];
    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        thrusts[i] = _out[i] - zero_out;
    }

    // rotors under Tiltrotor forward-thrust control ignore the
    // attitude mix; the plane's control surfaces stabilize the
    // vehicle
    const bool fwd_active = _fwd_mask != 0 && AP_HAL::millis() - _fwd_mask_ms < 100;
    if (fwd_active) {
        const float fwd_thrust = (armed() && get_interlock()) ? _fwd_thrust * (1.0f - zero_out) : 0.0f;
        for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
            if (_fwd_mask & (1U<<i)) {
                thrusts[i] = fwd_thrust;
            }
        }
    }

    if (_failed_mask != 0) {
        // a rotor with a failed drive motor produces no thrust.
        // Transferring its demand to the opposite corner with
        // inverted sign preserves the roll and pitch moments, with
        // the lost collective lift made up by the altitude
        // controller raising the demand on the working diagonal
        static const uint8_t opposite[AP_MOTORS_HELI_QUAD_NUM_MOTORS] = {1, 0, 3, 2};
        for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
            if (_failed_mask & (1U<<i)) {
                thrusts[opposite[i]] -= thrusts[i];
                thrusts[i] = 0.0f;
            }
        }
    }

    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        const float thrust = thrusts[i];

        // rotor speed which would centre the blade pitch at
        // _coll_trim. Negative thrust recruits rotor speed only when
        // rebalancing the moments of a failed drive motor - that is
        // its purpose, and a railed descent demand in normal flight
        // must not spool the rotors to full speed at full negative
        // pitch, where a winged airframe can settle into a powered
        // climb it never descends from. While landed the low
        // collective must not spool the rotors up at all
        float sizing_thrust = MAX(thrust, 0.0f);
        if (_failed_mask != 0 && !_heliflags.land_complete) {
            sizing_thrust = fabsf(thrust);
        }
        float speed_for_trim = 1.0f;
        if (is_positive(_coll_trim)) {
            speed_for_trim = sqrtf(sizing_thrust / _coll_trim);
        }

        // in fixed-wing flight a raised floor acts as a cruise
        // governor, holding blade-pitch authority in reserve
        float speed_min = _speed_min;
        if (fwd_active && (_fwd_mask & (1U<<i)) && is_positive(_speed_min_fw)) {
            speed_min = _speed_min_fw;
        }
        speed_for_trim = constrain_float(speed_for_trim, speed_min, 1.0f);

        // the speed demand follows slowly; the drive motors only see
        // the macro-level changes in thrust
        if (is_positive(_speed_tc)) {
            const float alpha = constrain_float(_dt_s / _speed_tc, 0.0f, 1.0f);
            _speed_demand[i] += (speed_for_trim - _speed_demand[i]) * alpha;
        } else {
            _speed_demand[i] = speed_for_trim;
        }
        _speed_demand[i] = constrain_float(_speed_demand[i], speed_min, 1.0f);

        // model the rotor's lagging response to the commanded speed
        if (is_positive(_rotor_tc)) {
            const float alpha = constrain_float(_dt_s / _rotor_tc, 0.0f, 1.0f);
            _rotor_speed_model[i] += (esc_output(i) - _rotor_speed_model[i]) * alpha;
        } else {
            _rotor_speed_model[i] = esc_output(i);
        }

        // prefer measured rotor speed from the drive motor's
        // telemetry, falling back to the model. ESC telemetry is
        // indexed by output channel, which need not equal the motor
        // function number (on a quadplane the drive motors sit above
        // the plane's control surfaces)
        _rotor_speed_est[i] = _rotor_speed_model[i];
#if HAL_WITH_ESC_TELEM
        float rpm;
        uint8_t telem_chan;
        if (is_positive(_rpm_max) &&
            SRV_Channels::find_channel(SRV_Channels::get_motor_function(AP_MOTORS_HELI_QUAD_DDVP_MOTOR_OFFSET+i), telem_chan) &&
            AP::esc_telem().get_rpm(telem_chan, rpm)) {
            const float measured = constrain_float(rpm / _rpm_max, 0.0f, 1.2f);
            _rotor_speed_est[i] = measured;

            // a rotor well below its expected speed has lost its
            // drive motor
            const uint32_t now_ms = AP_HAL::millis();
            if (armed() &&
                (_failed_mask & (1U<<i)) == 0 &&
                _rotor_speed_model[i] > 0.25f &&
                measured < 0.6f * _rotor_speed_model[i]) {
                if (_underspeed_ms[i] == 0) {
                    _underspeed_ms[i] = now_ms;
                } else if (now_ms - _underspeed_ms[i] > 200) {
                    _failed_mask |= 1U<<i;
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "DRSC: motor %u failed", unsigned(i+1));
                }
            } else {
                _underspeed_ms[i] = 0;
            }
        }
#endif  // HAL_WITH_ESC_TELEM

        // blade pitch supplies the demanded thrust at the modelled
        // rotor speed. Without a failed rotor to rebalance, negative
        // blade pitch is only useful for attitude moments: deep
        // negative collective can sail a winged airframe into a
        // powered glide that a railed descent demand never breaks
        // out of
        const float speed_sq = sq(MAX(_rotor_speed_est[i], 0.1f));
        const float blade_pitch = thrust / speed_sq;
        const float pitch_floor = (_failed_mask != 0) ? -1.0f : MAX(zero_out - 0.2f, -1.0f);
        _out[i] = constrain_float(zero_out + blade_pitch, pitch_floor, 1.0f);
    }
}

// forward-thrust control of tilted rotors in fixed-wing flight,
// replacing their attitude-mixed demands in move_actuators. The
// rudder differential is not applied: vectored yaw acts through the
// tilt servos rather than differential rotor thrust
void AP_MotorsHeli_Quad_DDVP::output_motor_mask(float thrust, uint32_t mask, float rudder_dt)
{
    _fwd_mask = mask;
    _fwd_thrust = constrain_float(thrust, 0.0f, 1.0f);
    _fwd_mask_ms = AP_HAL::millis();
}

void AP_MotorsHeli_Quad_DDVP::output_to_motors()
{
    AP_MotorsHeli_Quad::output_to_motors();

    if (!initialised_ok()) {
        return;
    }

    if (!armed() && _failed_mask != 0) {
        // failures are reassessed on the next flight
        _failed_mask = 0;
        memset(_underspeed_ms, 0, sizeof(_underspeed_ms));
    }

    // output the per-rotor drive motor speeds
    for (uint8_t i=0; i<AP_MOTORS_HELI_QUAD_NUM_MOTORS; i++) {
        rc_write(AP_MOTORS_MOT_1+AP_MOTORS_HELI_QUAD_DDVP_MOTOR_OFFSET+i, 1000 + esc_output(i) * 1000);
    }
}
