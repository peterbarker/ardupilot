'''
Plane autotest: FenceAltCeilFloor

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class FenceAltCeilFloor(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def FenceAltCeilFloor(self):
        '''Tests the fence ceiling and floor'''
        self.set_parameters({
            "FENCE_TYPE": 9,     # Set fence type to max and min alt
            "FENCE_ACTION": 0,   # Set action to report
            "FENCE_ALT_MAX": 200,
            "FENCE_ALT_MIN": 100,
        })

        # Grab Home Position
        self.wait_ready_to_arm()
        startpos = self.get_location()
        startpos_alt_amsl = startpos.get_alt_m(AltFrame.ABSOLUTE)

        cruise_alt = 150
        self.takeoff(cruise_alt)

        # note that while we enable the fence here, since the action
        # is set to report-only the fence continues to show as
        # not-enabled in the assert calls below
        self.do_fence_enable()

        self.progress("Fly above ceiling and check for breach")
        self.change_altitude(startpos_alt_amsl + cruise_alt + 80)
        self.assert_fence_sys_status(True, False, False)

        self.progress("Return to cruise alt")
        self.change_altitude(startpos_alt_amsl + cruise_alt)

        self.progress("Ensure breach has clearned")
        self.assert_fence_sys_status(True, False, True)

        self.progress("Fly below floor and check for breach")
        self.change_altitude(startpos_alt_amsl + cruise_alt - 80)

        self.progress("Ensure breach has clearned")
        self.assert_fence_sys_status(True, False, False)

        self.do_fence_disable()

        self.fly_home_land_and_disarm(timeout=150)
