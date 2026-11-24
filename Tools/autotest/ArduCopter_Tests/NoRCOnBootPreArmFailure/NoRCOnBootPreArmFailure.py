'''
Copter autotest: NoRCOnBootPreArmFailure

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class NoRCOnBootPreArmFailure(TestGroup):
    @testcase(bucket="CopterTests1e")
    def NoRCOnBootPreArmFailure(self):
        '''Ensure we can't arm with no RC on boot if THR_FS_VALUE set'''
        self.context_push()
        for rc_failure_mode in 1, 2:
            self.set_parameters({
                "SIM_RC_FAIL": rc_failure_mode,
            })
            self.reboot_sitl()
            if rc_failure_mode == 1:
                self.assert_prearm_failure("RC not found",
                                           other_prearm_failures_fatal=False)
            elif rc_failure_mode == 2:
                self.assert_prearm_failure("Throttle below failsafe",
                                           other_prearm_failures_fatal=False)
        self.context_pop()
        self.reboot_sitl()
