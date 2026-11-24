'''
Copter autotest: TakeoffCheck

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TakeoffCheck(TestGroup):
    # Tests the motor failsafe
    @testcase(bucket="CopterTests1d")
    def TakeoffCheck(self):
        '''Test takeoff check'''

        self.start_subtest("Test blocking doesn't occur with in-range RPM")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            'SIM_ESC_TELEM': 1,
            'SIM_ESC_ARM_RPM': 1000,
            'TKOFF_RPM_MIN': 900,
            'TKOFF_RPM_MAX': 1100,
        })
        self.takeoff(10, mode="LOITER")
        self.land_and_disarm()
        # ensure no spurious takeoff blocked warnings during spoolup
        for m in self.context_collection('STATUSTEXT'):
            if "Takeoff blocked" in m.text:
                raise NotAchievedException("Spurious takeoff blocked message: %s" % m.text)
        self.context_pop()

        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            'SIM_ESC_TELEM': 1,
            'SIM_ESC_ARM_RPM': 500,
            'TKOFF_RPM_MIN': 1000,
        })

        self.test_takeoff_check_mode("STABILIZE")
        self.test_takeoff_check_mode("ACRO")
        self.test_takeoff_check_mode("LOITER")
        self.test_takeoff_check_mode("ALT_HOLD")
        # self.test_takeoff_check_mode("FLOWHOLD")
        self.test_takeoff_check_mode("GUIDED", True)
        self.test_takeoff_check_mode("POSHOLD")
        # self.test_takeoff_check_mode("SPORT")

        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            'SIM_ESC_TELEM': 1,
            'TKOFF_RPM_MIN': 1,
            'TKOFF_RPM_MAX': 3,
        })
        self.test_takeoff_check_mode("STABILIZE")
        self.test_takeoff_check_mode("ACRO")
        self.test_takeoff_check_mode("LOITER")
        self.test_takeoff_check_mode("ALT_HOLD")
        # self.test_takeoff_check_mode("FLOWHOLD")
        self.test_takeoff_check_mode("GUIDED", True)
        self.test_takeoff_check_mode("POSHOLD")
        # self.test_takeoff_check_mode("SPORT")
