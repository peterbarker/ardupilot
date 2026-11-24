'''
Copter autotest: MAV_CMD_CONDITION_YAW

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_CONDITION_YAW(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MAV_CMD_CONDITION_YAW(self):
        '''Test response to MAV_CMD_CONDITION_YAW via mavlink'''
        self.context_push()
        self._MAV_CMD_CONDITION_YAW(self.run_cmd_int)
        self.context_pop()
        self.context_push()
        self._MAV_CMD_CONDITION_YAW(self.run_cmd)
        self.context_pop()
