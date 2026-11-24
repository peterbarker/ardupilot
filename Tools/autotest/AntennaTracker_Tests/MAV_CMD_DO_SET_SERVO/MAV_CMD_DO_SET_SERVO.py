'''
Tracker autotest: MAV_CMD_DO_SET_SERVO

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_SET_SERVO(TestGroup):
    @testcase(bucket="Tracker")
    def MAV_CMD_DO_SET_SERVO(self):
        '''Test SERVOTEST mode'''
        self.change_mode(0) # "MANUAL"
        # magically changes to SERVOTEST (3)
        for method in self.run_cmd, self.run_cmd_int:
            for value in 1900, 1200:
                channel = 1
                method(
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    p1=channel,
                    p2=value,
                    timeout=1,
                )
                self.wait_servo_channel_value(channel, value)
            for value in 1300, 1670:
                channel = 2
                method(
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    p1=channel,
                    p2=value,
                    timeout=1,
                )
                self.wait_servo_channel_value(channel, value)
