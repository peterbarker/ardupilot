'''
Plane autotest: MAV_CMD_DO_FLIGHTTERMINATION

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_FLIGHTTERMINATION(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def MAV_CMD_DO_FLIGHTTERMINATION(self):
        '''test MAV_CMD_DO_FLIGHTTERMINATION works on Plane'''
        self._MAV_CMD_DO_FLIGHTTERMINATION(self.run_cmd)
        self._MAV_CMD_DO_FLIGHTTERMINATION(self.run_cmd_int)
