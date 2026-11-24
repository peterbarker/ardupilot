'''
Plane autotest: GuidedRejectOutsideFence

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class GuidedRejectOutsideFence(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def GuidedRejectOutsideFence(self):
        '''ensure a GUIDED destination outside the fence is rejected'''
        self.set_parameters({
            "FENCE_TYPE": 2,     # circle only
            "FENCE_RADIUS": 1000,
            "FENCE_ACTION": 0,   # report only so a breach does not change mode
            "FENCE_ENABLE": 1,
        })
        self.takeoff(alt=50, mode='TAKEOFF')
        self.change_mode("GUIDED")
        # the fence is centered on home, so reference offsets from home
        home = self.home_position_as_location()

        inside_loc = self.offset_location_ne(home, 100, 0)
        outside_loc = self.offset_location_ne(home, 2000, 0)

        self.start_subtest("Reposition inside the fence should be accepted")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p5=int(inside_loc.lat * 1e7),
            p6=int(inside_loc.lng * 1e7),
            p7=50,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
        )

        self.start_subtest("Reposition outside the fence should be rejected")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p5=int(outside_loc.lat * 1e7),
            p6=int(outside_loc.lng * 1e7),
            p7=50,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )

        self.fly_home_land_and_disarm(timeout=180)
