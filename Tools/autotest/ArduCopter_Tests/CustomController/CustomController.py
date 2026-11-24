'''
Copter autotest: CustomController

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CustomController(TestGroup):
    @testcase(bucket="CopterTests2b")
    def CustomController(self, timeout=300):
        '''Test Custom Controller'''
        self.progress("Configure custom controller parameters")
        self.set_parameters({
            'CC_TYPE': 2,
            'CC_AXIS_MASK': 7,
            'RC6_OPTION': 109,
        })
        self.set_rc(6, 1000)
        self.reboot_sitl()

        if self.get_parameter("CC_TYPE") != 2 :
            raise NotAchievedException("Custom controller is not switched to PID backend.")

        # check if we can retrieve any param inside PID backend
        self.get_parameter("CC2_RAT_YAW_P")

        # takeoff in GPS mode and switch to CIRCLE
        self.takeoff(10, mode="LOITER", takeoff_throttle=2000)
        self.change_mode("CIRCLE")

        self.context_push()
        self.context_collect('STATUSTEXT')

        # switch custom controller on
        self.set_rc(6, 2000)
        self.wait_statustext("Custom controller is ON", check_context=True)

        # wait 20 second to see if the custom controller destabilize the aircraft
        if self.wait_altitude(7, 13, relative=True, minimum_duration=20) :
            raise NotAchievedException("Custom controller is not stable.")

        # switch custom controller off
        self.set_rc(6, 1000)
        self.wait_statustext("Custom controller is OFF", check_context=True)

        self.context_pop()
        self.do_RTL()
        self.progress("Custom controller test complete")
