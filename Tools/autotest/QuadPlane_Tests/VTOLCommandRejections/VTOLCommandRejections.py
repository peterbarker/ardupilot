'''
QuadPlane autotest: VTOLCommandRejections

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class VTOLCommandRejections(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def VTOLCommandRejections(self):
        '''check QuadPlane refuses VTOL commands it cannot act on'''
        DENIED = mavutil.mavlink.MAV_RESULT_DENIED
        FAILED = mavutil.mavlink.MAV_RESULT_FAILED

        self.change_mode('GUIDED')
        self.wait_ready_to_arm()

        self.start_subtest("NAV_TAKEOFF in a frame other than MAV_FRAME_LOCAL_OFFSET_NED")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            p7=10,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            want_result=DENIED,
        )

        self.start_subtest("NAV_TAKEOFF while disarmed")
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=10, want_result=FAILED)

        self.start_subtest("NAV_TAKEOFF outside GUIDED")
        self.change_mode('QHOVER')
        self.arm_vehicle()
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=10, want_result=FAILED)
        self.disarm_vehicle()

        self.start_subtest("NAV_TAKEOFF as COMMAND_INT")
        self.change_mode('GUIDED')
        self.arm_vehicle()
        takeoff_alt = 10
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            p7=-takeoff_alt,  # down is positive
            frame=mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
        )
        self.wait_altitude(takeoff_alt-1, takeoff_alt+1, relative=True, minimum_duration=5, timeout=60)

        self.start_subtest("NAV_TAKEOFF while flying")
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=20, want_result=FAILED)

        self.start_subtest("DO_VTOL_TRANSITION outside AUTO")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
            p1=mavutil.mavlink.MAV_VTOL_STATE_FW,
            want_result=FAILED,
        )

        self.change_mode('QLAND')
        self.wait_disarmed(timeout=120)
