'''
Copter autotest: GPSForYawCompassLearn

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GPSForYawCompassLearn(TestGroup):
    @testcase(bucket="CopterTests2d")
    def GPSForYawCompassLearn(self):
        '''Moving baseline GPS yaw - with compass learning'''
        self.context_push()
        self.load_default_params_file("copter-gps-for-yaw.parm")
        self.set_parameter("EK3_SRC1_YAW", 3)  # GPS with compass fallback
        self.reboot_sitl()

        self.wait_gps_fix_type_gte(6, message_type="GPS2_RAW", verbose=True)

        self.wait_ready_to_arm()

        self.takeoff(10, mode='GUIDED')
        tstart = self.get_sim_time()
        compass_learn_set = False
        while True:
            delta_t = self.get_sim_time_cached() - tstart
            if delta_t > 30:
                break
            if not compass_learn_set and delta_t > 10:
                self.set_parameter("COMPASS_LEARN", 3)
                compass_learn_set = True

            self.check_attitudes_match()
            self.delay_sim_time(1, reason="poll interval")

        self.do_RTL()
        self.context_pop()
        self.reboot_sitl()
