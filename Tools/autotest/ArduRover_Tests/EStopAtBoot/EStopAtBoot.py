'''
Rover autotest: EStopAtBoot

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class EStopAtBoot(TestGroup):
    @testcase(bucket="Rover")
    def EStopAtBoot(self):
        '''Ensure EStop prevents arming when asserted at boot time'''
        self.context_push()
        self.set_parameters({
            "RC9_OPTION": 31,
        })
        self.set_rc(9, 2000)
        self.reboot_sitl()
        self.assert_prearm_failure(
            "Motors Emergency Stopped",
            other_prearm_failures_fatal=False)
        self.context_pop()
        self.reboot_sitl()
