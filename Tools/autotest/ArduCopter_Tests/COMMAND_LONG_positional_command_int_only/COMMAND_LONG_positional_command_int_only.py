'''
Copter autotest: COMMAND_LONG_positional_command_int_only

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class COMMAND_LONG_positional_command_int_only(TestGroup):
    @testcase(bucket="CopterTests2b")
    def COMMAND_LONG_positional_command_int_only(self):
        '''check positional commands with no COMMAND_LONG frame are refused as COMMAND_INT-only'''
        for command in (
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                mavutil.mavlink.MAV_CMD_EXTERNAL_POSITION_ESTIMATE,
                mavutil.mavlink.MAV_CMD_DO_SET_GLOBAL_ORIGIN,
        ):
            self.run_cmd(
                command,
                want_result=mavutil.mavlink.MAV_RESULT_COMMAND_INT_ONLY,
            )
