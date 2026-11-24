'''
Copter autotest: MAV_CMD_AIRFRAME_CONFIGURATION

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_AIRFRAME_CONFIGURATION(TestGroup):
    @testcase(bucket="CopterTests2b")
    def MAV_CMD_AIRFRAME_CONFIGURATION(self):
        '''deploy/retract landing gear using mavlink command'''
        self.context_push()
        self.set_parameters({
            "LGR_ENABLE": 1,
            "SERVO10_FUNCTION": 29,
            "SERVO10_MIN": 1001,
            "SERVO10_MAX": 1999,
        })
        self.reboot_sitl()

        # starts loose:
        self.wait_servo_channel_value(10, 0)

        # 0 is down:
        self.start_subtest("Put gear down")
        self.run_cmd(mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION, p2=0)
        self.wait_servo_channel_value(10, 1999)

        # 1 is up:
        self.start_subtest("Put gear up")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION, p2=1)
        self.wait_servo_channel_value(10, 1001)

        # 0 is down:
        self.start_subtest("Put gear down")
        self.run_cmd(mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION, p2=0)
        self.wait_servo_channel_value(10, 1999)

        self.context_pop()
        self.reboot_sitl()
