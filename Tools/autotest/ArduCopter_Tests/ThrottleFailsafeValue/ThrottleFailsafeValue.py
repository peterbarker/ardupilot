'''
Copter autotest: ThrottleFailsafeValue

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ThrottleFailsafeValue(TestGroup):
    @testcase(bucket="CopterTests1c")
    def ThrottleFailsafeValue(self):
        '''check the throttle failsafe triggers on FS_THR_VALUE itself'''
        self.set_parameters({
            "FS_THR_ENABLE": 1,
            "FS_THR_VALUE": 975,
        })

        self.start_subtest("one above the failsafe value is not a failsafe")
        self.set_rc(3, 976)
        self.wait_ready_to_arm()

        self.start_subtest("the failsafe value itself is a failsafe")
        self.context_collect('STATUSTEXT')
        self.set_rc(3, 975)
        self.wait_statustext('Radio Failsafe', check_context=True)
        self.assert_prearm_failure("Throttle below failsafe",
                                   other_prearm_failures_fatal=False)

        self.start_subtest("recovers when the throttle comes back up")
        self.set_rc(3, 976)
        self.wait_statustext('Radio Failsafe Cleared', check_context=True)
        self.context_stop_collecting('STATUSTEXT')
        self.wait_ready_to_arm()
