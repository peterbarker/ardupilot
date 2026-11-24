'''
Plane autotest: MAV_CMD_DO_AUTOTUNE_ENABLE

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_AUTOTUNE_ENABLE(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def MAV_CMD_DO_AUTOTUNE_ENABLE(self):
        '''test enabling autotune via mavlink'''
        self.context_collect('STATUSTEXT')
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_AUTOTUNE_ENABLE, p1=1)
        self.wait_statustext('Started autotune', check_context=True)
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_AUTOTUNE_ENABLE, p1=0)
        self.wait_statustext('Stopped autotune', check_context=True)
