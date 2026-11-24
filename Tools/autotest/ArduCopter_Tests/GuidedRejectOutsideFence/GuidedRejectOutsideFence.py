'''
Copter autotest: GuidedRejectOutsideFence

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class GuidedRejectOutsideFence(TestGroup):
    @testcase(bucket="CopterTests2a")
    def GuidedRejectOutsideFence(self):
        '''ensure a GUIDED destination outside the fence is rejected'''
        self.set_parameters({
            "FENCE_TYPE": 2,    # circle only
            "FENCE_RADIUS": 50,
            "FENCE_ACTION": 0,  # report only so a breach does not change mode
            "FENCE_ENABLE": 1,
        })
        self.takeoff(10, mode='GUIDED')
        here = self.get_location()

        inside_loc = self.offset_location_ne(here, 10, 0)
        outside_loc = self.offset_location_ne(here, 200, 0)

        self.start_subtest("Reposition inside the fence should be accepted")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            x=int(inside_loc.lat * 1e7),
            y=int(inside_loc.lng * 1e7),
            z=here.get_alt_m(AltFrame.ABSOLUTE),
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
        )

        self.start_subtest("Reposition outside the fence should be rejected")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            x=int(outside_loc.lat * 1e7),
            y=int(outside_loc.lng * 1e7),
            z=here.get_alt_m(AltFrame.ABSOLUTE),
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )

        self.do_RTL()
