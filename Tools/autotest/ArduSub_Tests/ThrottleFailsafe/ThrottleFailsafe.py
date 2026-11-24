'''
Sub autotest: ThrottleFailsafe

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ThrottleFailsafe(TestGroup):
    # Tests actions and logic behind the radio failsafe
    @testcase(bucket="Sub")
    def ThrottleFailsafe(self):
        '''Test RC and RC Failsafe'''
        # disable GCS and enable RC
        self.set_parameter('FS_GCS_ENABLE', 0)
        self.set_parameter('RC_PROTOCOLS', 1)
        self.set_heartbeat_rate(0)
        # Trigger an RC failure with the failsafe disabled. Verify no action taken.
        self.start_subtest("Radio failsafe disabled test: FS_THR_ENABLE=0 should take no failsafe action")
        self.set_parameter('FS_THR_ENABLE', 0)
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_ready_to_arm()
        # Switch RC failsafe action to SURFACE, should take action since we are in RC failsafe
        self.set_parameter('FS_THR_ENABLE', 2)
        self.wait_mode("SURFACE")
        self.progress("Completed Radio failsafe disabled test")
