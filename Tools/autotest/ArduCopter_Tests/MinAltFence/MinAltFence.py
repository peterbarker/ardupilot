'''
Copter autotest: MinAltFence

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class MinAltFence(TestGroup):
    # fly_alt_min_fence_test - fly down until you hit the fence floor
    def min_alt_fence_frame(self, frame, alt, terrain=0):
        '''Test Min Alt Fence'''

        if terrain == 1:
            self.set_parameters({
                "TERRAIN_ENABLE": terrain,
                "SIM_TERRAIN": terrain,
            })
            self.install_terrain_handlers_context()

        self.takeoff(30, mode="LOITER", timeout=60)

        # enable fence, disable avoidance
        self.set_parameters({
            "AVOID_ENABLE": 0,
            "FENCE_ENABLE" : 1,
            "FENCE_TYPE": 8,
            "FENCE_ALT_MIN": alt,
            "FENCE_ALT_MIN_TP": frame,
        })

        self.change_alt(30)

        # Activate the floor fence
        # TODO this test should run without requiring this
        self.do_fence_enable()

        # first east
        self.progress("turn east")
        self.set_rc(4, 1580)
        self.wait_heading(160, timeout=60)
        self.set_rc(4, 1500)

        # fly forward (east) at least 20m
        self.set_rc(2, 1100)
        self.wait_distance(20)

        # stop flying forward and start flying down:
        self.set_rc_from_map({
            2: 1500,
            3: 1200,
        })

        # wait for fence to trigger
        self.wait_mode('RTL', timeout=120)

        self.wait_rtl_complete()

        # Disable the fence using mavlink command to ensure cleaned up SITL state
        self.do_fence_disable()

        self.zero_throttle()

    @testcase(bucket="CopterTests2c")
    def MinAltFence(self):
        '''Test Min Alt Fences'''
        self.poll_home_position(quiet=False)
        home_loc = self.get_location()
        self.set_home(home_loc)

        self.min_alt_fence_frame(0, home_loc.get_alt_m(AltFrame.ABSOLUTE) + 20)    # absolute
        self.min_alt_fence_frame(3, 20, 1) # above terrain
        self.min_alt_fence_frame(2, 20)    # above origin

        # set origin below home alt - test should still pass
        nz = self.offset_location_up(home_loc, -20)
        self.set_origin(nz)
        self.set_parameters({
            "SIM_GPS1_ENABLE": 1,
        })
        self.min_alt_fence_frame(1, 20)    # above home
