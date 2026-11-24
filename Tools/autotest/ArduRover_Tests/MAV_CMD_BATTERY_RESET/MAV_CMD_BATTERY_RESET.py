'''
Rover autotest: MAV_CMD_BATTERY_RESET

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_BATTERY_RESET(TestGroup):
    @testcase(bucket="Rover")
    def MAV_CMD_BATTERY_RESET(self):
        '''manipulate battery levels with MAV_CMD_BATTERY_RESET'''
        for (run_cmd, value) in (self.run_cmd, 56), (self.run_cmd_int, 97):
            run_cmd(
                mavutil.mavlink.MAV_CMD_BATTERY_RESET,
                p1=65535,  # battery mask
                p2=value,
            )
            self.assert_received_message_field_values('BATTERY_STATUS', {
                "battery_remaining": value,
            }, {
                "poll": True,
            })
