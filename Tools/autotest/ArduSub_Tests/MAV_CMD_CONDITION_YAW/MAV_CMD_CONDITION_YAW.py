'''
Sub autotest: MAV_CMD_CONDITION_YAW

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_CONDITION_YAW(TestGroup):
    @testcase(bucket="Sub")
    def MAV_CMD_CONDITION_YAW(self):
        '''ensure vehicle yaws according to GCS command'''
        self._MAV_CMD_CONDITION_YAW(self.run_cmd)
        self._MAV_CMD_CONDITION_YAW(self.run_cmd_int)
