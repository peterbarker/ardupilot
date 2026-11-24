'''
QuadPlane autotest: HIGH_LATENCY2_VTOL

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import quaternion

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class HIGH_LATENCY2_VTOL(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def HIGH_LATENCY2_VTOL(self):
        '''check HIGH_LATENCY2 navigation targets in VTOL modes'''
        self.takeoff(60, mode='QHOVER', timeout=60)
        self.change_mode('QLOITER')
        self.delay_sim_time(5, reason="vehicle to settle")

        self.start_subtest("target_heading is the attitude controller's yaw target")
        # slow the yaw response so the yaw target runs well ahead of the vehicle:
        yaw_gains = self.get_parameters(["Q_A_RAT_YAW_P", "Q_A_RAT_YAW_I"])
        self.set_parameters({name: value * 0.1 for name, value in yaw_gains.items()})
        self.set_rc(4, 2000)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 60:
                raise NotAchievedException("Did not get a yaw target well ahead of the vehicle's yaw")
            frame = self.get_messages_frame(['HIGH_LATENCY2', 'ATTITUDE_TARGET', 'ATTITUDE'], timeout=10)
            target_yaw = math.degrees(quaternion.Quaternion(frame['ATTITUDE_TARGET'].q).euler[2]) % 360
            yaw = math.degrees(frame['ATTITUDE'].yaw) % 360
            # yaw targets beyond 180 degrees are negative in the
            # controller and are not reported correctly, so stay below:
            if not 20 < target_yaw < 160 or self.heading_delta(target_yaw, yaw) < 15:
                continue
            # target_heading is in units of 2 degrees:
            target_heading = frame['HIGH_LATENCY2'].target_heading * 2
            self.progress(f"target_heading={target_heading} yaw target={target_yaw:.1f} yaw={yaw:.1f}")
            if self.heading_delta(target_heading, target_yaw) > 5:
                raise NotAchievedException(f"target_heading {target_heading} is not the yaw target {target_yaw:.1f}")
            break
        self.set_rc(4, 1500)
        self.set_parameters(yaw_gains)

        self.start_subtest("target_altitude is the position controller's altitude target")
        # without enough lift the vehicle sinks below its altitude target:
        self.set_parameters({
            "SIM_ENGINE_FAIL": 0xF0,  # VTOL motors, servos 5 to 8
            "SIM_ENGINE_MUL": 0.4,
        })
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 30:
                raise NotAchievedException("Did not get an altitude target well above the vehicle")
            frame = self.get_messages_frame(['HIGH_LATENCY2', 'GLOBAL_POSITION_INT', 'NAV_CONTROLLER_OUTPUT'], timeout=10)
            alt_error = frame['NAV_CONTROLLER_OUTPUT'].alt_error
            if alt_error < 1.8:
                continue
            want_altitude = frame['GLOBAL_POSITION_INT'].alt * 0.001 + alt_error
            target_altitude = frame['HIGH_LATENCY2'].target_altitude
            self.progress(f"target_altitude={target_altitude} want={want_altitude:.2f} alt_error={alt_error:.2f}")
            # target_altitude is truncated to whole metres:
            if not -0.5 < want_altitude - target_altitude < 1.5:
                raise NotAchievedException(f"target_altitude {target_altitude} is not the altitude target {want_altitude:.2f}")
            break
        self.set_parameter("SIM_ENGINE_MUL", 1)

        self.change_mode('QLAND')
        self.wait_disarmed(timeout=120)
