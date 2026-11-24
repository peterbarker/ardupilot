'''
Rover autotest: MAV_CMD_GET_HOME_POSITION

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_GET_HOME_POSITION(TestGroup):
    def _MAV_CMD_GET_HOME_POSITION(self, run_cmd):
        '''test handling of mavlink command MAV_CMD_GET_HOME_POSITION'''
        self.context_collect('HOME_POSITION')
        run_cmd(mavutil.mavlink.MAV_CMD_GET_HOME_POSITION)
        self.assert_receive_message('HOME_POSITION', check_context=True)

    @testcase(bucket="Rover")
    def MAV_CMD_GET_HOME_POSITION(self):
        '''test handling of mavlink command MAV_CMD_GET_HOME_POSITION'''
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self._MAV_CMD_GET_HOME_POSITION(self.run_cmd)
        self._MAV_CMD_GET_HOME_POSITION(self.run_cmd_int)
