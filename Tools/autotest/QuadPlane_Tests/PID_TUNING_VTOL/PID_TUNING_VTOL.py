'''
QuadPlane autotest: PID_TUNING_VTOL

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PID_TUNING_VTOL(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def PID_TUNING_VTOL(self):
        '''check PID_TUNING is sent from the VTOL controllers in VTOL modes'''
        self.set_parameter("GCS_PID_MASK", 1 | 2 | 4 | 32)  # roll, pitch, yaw, accz
        self.context_set_message_rate_hz('EXTENDED_SYS_STATE', 10)
        fixed_wing_axes = set([
            mavutil.mavlink.PID_TUNING_ROLL,
            mavutil.mavlink.PID_TUNING_PITCH,
            mavutil.mavlink.PID_TUNING_YAW,
        ])
        vtol_axes = fixed_wing_axes | set([mavutil.mavlink.PID_TUNING_ACCZ])

        self.takeoff(20, mode='QHOVER')
        axes = self.received_pid_tuning_axes()
        if axes != vtol_axes:
            raise NotAchievedException(f"QHOVER: want axes {sorted(vtol_axes)} got {sorted(axes)}")

        # the VTOL rate controllers report the vehicle's body rate, in
        # radians/second, as "achieved"; the fixed-wing controllers
        # would report something else.  Slowing the rate controller
        # separates "desired" from "achieved" so the two can't be
        # confused.  ATTITUDE and PID_TUNING are streamed
        # independently, so compare each PID_TUNING against the body
        # rates in the ATTITUDE messages either side of it:
        self.context_set_message_rate_hz('PID_TUNING', 20)
        self.context_set_message_rate_hz('ATTITUDE', 50)
        for axis, rate_field, channel, gain_prefix in [
                (mavutil.mavlink.PID_TUNING_ROLL, 'rollspeed', 1, 'Q_A_RAT_RLL_'),
                (mavutil.mavlink.PID_TUNING_PITCH, 'pitchspeed', 2, 'Q_A_RAT_PIT_'),
                (mavutil.mavlink.PID_TUNING_YAW, 'yawspeed', 4, 'Q_A_RAT_YAW_'),
        ]:
            self.start_subtest(f"PID_TUNING axis {axis} is the VTOL rate controller")
            gains = self.get_parameters([gain_prefix + g for g in ('P', 'I', 'D')])
            rates = []
            pid_samples = []  # (desired, achieved, index of the following entry in rates)

            def collect(mav, m):
                if m.get_type() == 'ATTITUDE':
                    rates.append(getattr(m, rate_field))
                elif m.get_type() == 'PID_TUNING' and m.axis == axis:
                    pid_samples.append((m.desired, m.achieved, len(rates)))

            self.set_parameters({name: value * 0.25 for name, value in gains.items()})
            self.context_push()
            try:
                self.install_message_hook_context(collect)
                self.set_rc(channel, 1900)
                self.delay_sim_time(1, reason="vehicle to rotate")
                self.set_rc(channel, 1500)
                self.delay_sim_time(2, reason="vehicle to settle")
            finally:
                self.context_pop()
                self.set_parameters(gains)

            samples = []  # (desired, achieved, lower rate, upper rate)
            for (desired, achieved, following) in pid_samples:
                if following == 0 or following >= len(rates):
                    continue
                lower = min(rates[following-1], rates[following])
                upper = max(rates[following-1], rates[following])
                if max(abs(lower), abs(upper)) > 0.5:
                    samples.append((desired, achieved, lower, upper))

            def distance_outside(value, lower, upper):
                return max(lower - value, value - upper, 0)

            self.progress(f"(desired, achieved, rate range) samples: {samples}")
            if len(samples) < 5:
                raise NotAchievedException(f"Vehicle did not rotate on axis {axis}")
            matching = [s for s in samples if distance_outside(s[1], s[2], s[3]) < 0.05]
            if len(matching) < 0.9 * len(samples):
                raise NotAchievedException(f"PID_TUNING axis {axis} achieved does not follow the body rate")
            separated = [s for s in matching if distance_outside(s[0], s[2], s[3]) > 0.3]
            if len(separated) < 3:
                raise NotAchievedException(f"PID_TUNING axis {axis} desired was never distinct from the body rate")
            self.delay_sim_time(2, reason="vehicle to settle")

        self.change_mode('FBWA')
        self.set_rc(3, 1900)
        self.wait_extended_sys_state(
            mavutil.mavlink.MAV_VTOL_STATE_FW,
            mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
            timeout=60,
        )
        axes = self.received_pid_tuning_axes()
        if axes != fixed_wing_axes:
            raise NotAchievedException(f"FBWA: want axes {sorted(fixed_wing_axes)} got {sorted(axes)}")
        self.set_rc(3, 1500)

        self.change_mode('QLAND')
        self.wait_disarmed(timeout=120)
