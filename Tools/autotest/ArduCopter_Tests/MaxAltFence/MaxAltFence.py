'''
Copter autotest: MaxAltFence

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class MaxAltFence(TestGroup):
    # MaxAltFence - fly up until you hit the fence ceiling
    def max_alt_fence_frame(self, frame, alt, expected_breach_alt, terrain=0):
        self.start_subtest("Test Max Alt Fence in frame %u" % frame)

        # enable fence, disable avoidance
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "AVOID_ENABLE": 0,
            "FENCE_TYPE": 1,
            "FENCE_ALT_MAX_TP": frame,
            "FENCE_ALT_MAX": alt,
            "TERRAIN_ENABLE": terrain,
            "SIM_TERRAIN": terrain,
        })

        if terrain == 1:
            self.install_terrain_handlers_context()

        self.takeoff(10, mode="LOITER")
        """Hold loiter position."""
        self.change_alt(10)

        # first east
        self.progress("turning east")
        self.set_rc(4, 1580)
        self.wait_heading(160, timeout=60)
        self.set_rc(4, 1500)

        self.progress("flying east 20m")
        self.set_rc(2, 1100)
        self.wait_distance(20)

        self.progress("flying up")
        self.set_rc_from_map({
            2: 1500,
            3: 1800,
        })

        # wait for fence to trigger
        self.wait_mode('RTL', timeout=120)

        # verify breach occurred at expected altitude
        self.assert_altitude(expected_breach_alt, accuracy=5)

        self.wait_rtl_complete()

        self.zero_throttle()

    # MaxAltFence - fly up until you hit the fence ceiling
    @testcase(bucket="CopterTests1e")
    def MaxAltFence(self):
        '''Test Max Alt Fences'''
        self.poll_home_position(quiet=False)
        home_loc = self.get_location()
        origin_alt = home_loc.get_alt_m(AltFrame.ABSOLUTE)

        self.max_alt_fence_frame(0, origin_alt + 80, origin_alt + 80)      # absolute
        self.max_alt_fence_frame(1, 100, origin_alt + 100)                 # above home

        # set home 50m higher than origin to test that frame 2 uses origin not home
        self.set_home(self.offset_location_up(home_loc, 50))
        self.max_alt_fence_frame(2, 120, origin_alt + 120)                 # above origin

        self.max_alt_fence_frame(3, 90, origin_alt + 90, terrain=1)        # above terrain
