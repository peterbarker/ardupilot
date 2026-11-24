'''
Copter autotest: DO_WINCH

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class DO_WINCH(TestGroup):
    def _DO_WINCH(self, command):
        self.context_push()
        self.load_default_params_file("copter-winch.parm")
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.start_subtest("starts relaxed")
        self.wait_servo_channel_value(9, 0)

        self.start_subtest("rate control")
        command(
            mavutil.mavlink.MAV_CMD_DO_WINCH,
            p1=1,  # instance number
            p2=mavutil.mavlink.WINCH_RATE_CONTROL,  # command
            p3=0,  # length to release
            p4=1,  # rate in m/s
        )
        self.wait_servo_channel_value(9, 1900)

        self.start_subtest("relax")
        command(
            mavutil.mavlink.MAV_CMD_DO_WINCH,
            p1=1,  # instance number
            p2=mavutil.mavlink.WINCH_RELAXED,  # command
            p3=0,  # length to release
            p4=1,  # rate in m/s
        )
        self.wait_servo_channel_value(9, 0)

        self.start_subtest("hold but zero output")
        command(
            mavutil.mavlink.MAV_CMD_DO_WINCH,
            p1=1,  # instance number
            p2=mavutil.mavlink.WINCH_RATE_CONTROL,  # command
            p3=0,  # length to release
            p4=0,  # rate in m/s
        )
        self.wait_servo_channel_value(9, 1500)

        self.start_subtest("relax")
        command(
            mavutil.mavlink.MAV_CMD_DO_WINCH,
            p1=1,  # instance number
            p2=mavutil.mavlink.WINCH_RELAXED,  # command
            p3=0,  # length to release
            p4=1,  # rate in m/s
        )
        self.wait_servo_channel_value(9, 0)

        self.start_subtest("position")
        command(
            mavutil.mavlink.MAV_CMD_DO_WINCH,
            p1=1,  # instance number
            p2=mavutil.mavlink.WINCH_RELATIVE_LENGTH_CONTROL,  # command
            p3=2,  # length to release
            p4=1,  # rate in m/s
        )
        self.wait_servo_channel_value(9, 1900)
        self.wait_servo_channel_value(9, 1500, timeout=60)

        self.context_pop()
        self.reboot_sitl()

    @testcase(bucket="CopterTests2a")
    def DO_WINCH(self):
        '''test mavlink DO_WINCH command'''
        self._DO_WINCH(self.run_cmd_int)
        self._DO_WINCH(self.run_cmd)
