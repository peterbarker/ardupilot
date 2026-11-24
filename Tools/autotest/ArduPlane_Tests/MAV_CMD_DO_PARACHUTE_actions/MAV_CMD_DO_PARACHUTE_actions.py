'''
Plane autotest: MAV_CMD_DO_PARACHUTE_actions

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_PARACHUTE_actions(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def MAV_CMD_DO_PARACHUTE_actions(self):
        '''test enabling, disabling and repeated release of the parachute via mavlink'''
        self.setup_simulated_parachute()
        # releasing on the ground requires a vehicle which has never flown:
        self.reboot_sitl()
        self.context_collect('STATUSTEXT')

        self.start_subtest("disable")
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_PARACHUTE, p1=mavutil.mavlink.PARACHUTE_DISABLE)
        self.assert_parameter_value("CHUTE_ENABLED", 0)
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=mavutil.mavlink.PARACHUTE_RELEASE,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )
        self.wait_statustext("Parachute not enabled", check_context=True)

        self.start_subtest("enable")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_PARACHUTE, p1=mavutil.mavlink.PARACHUTE_ENABLE)
        self.assert_parameter_value("CHUTE_ENABLED", 1)

        self.start_subtest("action not in PARACHUTE_ACTION")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=3,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )

        self.start_subtest("release twice")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_PARACHUTE, p1=mavutil.mavlink.PARACHUTE_RELEASE)
        self.wait_servo_channel_value(9, 1300)
        self.wait_disarmed()
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=mavutil.mavlink.PARACHUTE_RELEASE,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )
        self.wait_statustext("Parachute already released", check_context=True)

        # the released state is only cleared by a reboot:
        self.reboot_sitl()
