'''
Plane autotest: FenceCircleExclusionAutoEnable

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class FenceCircleExclusionAutoEnable(TestGroup):
    @testcase(bucket="PlaneTests1a", speedup=20)
    def FenceCircleExclusionAutoEnable(self):
        '''Tests autolanding when alt min fence is enabled'''
        self.set_parameters({
            "FENCE_TYPE": 2,     # Set fence type to circle
            "FENCE_ACTION": 1,   # Set action to RTL
            "FENCE_AUTOENABLE": 2,
            "FENCE_ENABLE" : 0,
            "RTL_AUTOLAND" : 2,
        })

        fence_loc = self.home_position_as_location()
        self.location_offset_ne(fence_loc, 300, 0)

        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, {
                "radius" : 100,
                "loc" : fence_loc
            }
        )])

        self.takeoff(alt=50, mode='TAKEOFF')
        self.change_mode("FBWA")
        self.set_rc(3, 1100)    # lower throttle

        self.progress("Waiting for RTL")
        self.wait_mode('RTL')
        # Now check we can land
        self.fly_home_land_and_disarm()
