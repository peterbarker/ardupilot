'''
Rover autotest: MAV_CMD_DO_SEND_BANNER

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_SEND_BANNER(TestGroup):
    def _MAV_CMD_DO_SEND_BANNER(self, run_cmd):
        '''Get Banner'''
        self.context_push()
        self.context_collect('STATUSTEXT')
        run_cmd(mavutil.mavlink.MAV_CMD_DO_SEND_BANNER)
        self.wait_statustext("ArduRover", timeout=1, check_context=True)
        self.context_pop()

    # self.DriveBrake,  # disabled due to frequent failures
    @testcase(bucket=["Rover", "BalanceBot"])
    def MAV_CMD_DO_SEND_BANNER(self):
        '''test MAV_CMD_DO_SEND_BANNER'''
        self._MAV_CMD_DO_SEND_BANNER(self.run_cmd)
        self._MAV_CMD_DO_SEND_BANNER(self.run_cmd_int)
