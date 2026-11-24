'''
Rover autotest: JSONNoTimeSync

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class JSONNoTimeSync(TestGroup):
    @testcase(bucket="Rover")
    def JSONNoTimeSync(self):
        '''check SITL runs as fast as a JSON physics backend using no_time_sync'''
        self.context_set_speedup(1)
        with self.JSON_physics(fields={'no_time_sync': True}) as physics:
            # SIM_JSON makes the simulator AHRS the default
            self.assert_parameter_value("AHRS_EKF_TYPE", 10)

            # not held to the speedup
            ratio = self.JSON_sim_wall_ratio()
            if ratio < 5:
                raise NotAchievedException("SITL ran at %fx with SIM_SPEEDUP=1, expected no time sync" % ratio)

            # frame rate is not forced to the physics step
            sim_rate = self.get_parameter("SIM_RATE_HZ")
            frame_rate = self.JSON_frame_rate(physics)
            if frame_rate != sim_rate:
                raise NotAchievedException("frame_rate %.1f, expected SIM_RATE_HZ %u" % (frame_rate, sim_rate))
