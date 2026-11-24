'''
Plane autotest: TestRCCamera

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import PreconditionFailedException

from test_discovery import TestGroup
from test_discovery import testcase


class TestRCCamera(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def TestRCCamera(self):
        '''Test RC Option - Camera Trigger'''
        self.set_parameter("RC12_OPTION", 9) # CameraTrigger
        self.set_parameter("CAM1_TYPE", 1)   # Camera with servo trigger
        self.reboot_sitl() # needed for RC12_OPTION to take effect

        x = self.mav.messages.get("CAMERA_FEEDBACK", None)
        if x is not None:
            raise PreconditionFailedException("Receiving CAMERA_FEEDBACK?!")
        self.set_rc(12, 2000)
        x = self.assert_receive_message('CAMERA_FEEDBACK', timeout=10)
        self.set_rc(12, 1000)

        self.wait_ready_to_arm()

        original_alt = self.get_altitude()

        takeoff_alt = 30
        self.takeoff(takeoff_alt)
        self.set_rc(12, 2000)
        self.delay_sim_time(1, reason="camera trigger to register")
        self.set_rc(12, 1000)
        x = self.mav.messages.get("CAMERA_FEEDBACK", None)
        if abs(x.alt_rel - takeoff_alt) > 10:
            raise NotAchievedException("Bad relalt (want=%f vs got=%f)" % (takeoff_alt, x.alt_rel))
        if abs(x.alt_msl - (original_alt+30)) > 10:
            raise NotAchievedException("Bad absalt (want=%f vs got=%f)" % (original_alt+30, x.alt_msl))

        loc = self.get_location()
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p2=mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
            p5=int(loc.lat * 1e7),
            p6=int(loc.lng * 1e7),
            p7=x.alt_rel,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        expected_radius = 100
        self.wait_circling_point_with_radius(loc, expected_radius)

        self.context_collect('CAMERA_FEEDBACK')
        self.set_rc(12, 2000)
        self.delay_sim_time(1, reason="camera trigger to register")
        self.set_rc(12, 1000)
        self.assert_received_message_field_values('CAMERA_FEEDBACK', {
            "roll": math.degrees(self.assert_receive_message('ATTITUDE').roll),
        }, check_context=True, epsilon=5.0)

        self.fly_home_land_and_disarm()
