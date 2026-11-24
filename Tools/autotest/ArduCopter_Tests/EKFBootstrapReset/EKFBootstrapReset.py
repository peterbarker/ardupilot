'''
Copter autotest: EKFBootstrapReset

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class EKFBootstrapReset(TestGroup):
    @testcase(bucket="CopterTests2b")
    def EKFBootstrapReset(self):
        '''verify EKF reset aux switch is disarmed-only and preserves origin'''
        self.set_parameters({
            "RC8_OPTION": 187,  # EKF_RESET
        })
        self.reboot_sitl()

        self.wait_ready_to_arm()
        home = self.get_location()

        self.context_collect('STATUSTEXT')

        # disarmed: reset should succeed and re-bootstrap the cores
        self.set_rc(8, 2000)
        self.wait_statustext("EKF bootstrap reset performed", check_context=True, timeout=10)
        self.wait_statustext("EKF3 IMU. initialised", check_context=True, regex=True, timeout=10)
        self.set_rc(8, 1000)

        # take off and confirm the GUIDED position controller holds station
        # at the pre-reset location - if the origin had moved during the
        # reset, position hold would drive the vehicle away from home
        self.takeoff(10, mode='GUIDED')
        self.wait_location(home, accuracy=5, height_accuracy=None,
                           minimum_duration=10, timeout=30)

        # armed: reset should be refused
        self.set_rc(8, 2000)
        self.wait_statustext("EKF reset ignored: vehicle armed", check_context=True, timeout=10)
        self.set_rc(8, 1000)

        self.do_RTL()
