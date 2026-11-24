'''
Plane autotest: FenceBreachedChangeMode

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import WaitModeTimeout

from test_discovery import TestGroup
from test_discovery import testcase


class FenceBreachedChangeMode(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def FenceBreachedChangeMode(self):
        '''Tests manual mode change after fence breach, as set with FENCE_OPTIONS'''
        """ Attempts to change mode while a fence is breached.
            mode should change should fail if fence option bit is set"""
        self.set_parameters({
            "FENCE_ACTION": 1,
            "FENCE_TYPE": 4,
        })
        home_loc = self.get_location()
        locs = [
            Location.latlon_only(home_loc.lat - 0.001, home_loc.lng - 0.001),
            Location.latlon_only(home_loc.lat - 0.001, home_loc.lng + 0.001),
            Location.latlon_only(home_loc.lat + 0.001, home_loc.lng + 0.001),
            Location.latlon_only(home_loc.lat + 0.001, home_loc.lng - 0.001),
        ]
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, locs),
        ])
        self.delay_sim_time(1, reason="fence upload to complete")
        self.wait_ready_to_arm()
        self.takeoff(alt=50)
        self.change_mode("CRUISE")
        self.wait_distance(250, accuracy=15)

        self.progress("Enable fence and initiate fence action")
        self.do_fence_enable()
        self.assert_fence_enabled()
        self.wait_mode("RTL") # We should RTL because of fence breach

        self.progress("User mode change to cruise should retrigger fence action")
        try:
            # mode change should time out, 'WaitModeTimeout' exception is the desired result
            # can't wait too long or the vehicle will be inside fence and allow the mode change
            self.change_mode("CRUISE", timeout=10)
            raise NotAchievedException("Should not change mode in fence breach")
        except WaitModeTimeout:
            pass
        except Exception as e:
            raise e

        # enable mode change
        self.set_parameter("FENCE_OPTIONS", 0)
        self.progress("Check user mode change to LOITER is allowed")
        self.change_mode("LOITER")

        # Fly for 20 seconds and make sure still in LOITER mode
        self.delay_sim_time(20, reason="fence not to retrigger")
        if not self.mode_is("LOITER"):
            raise NotAchievedException("Fence should not re-trigger")

        # reset options parameter
        self.set_parameter("FENCE_OPTIONS", 1)

        self.progress("Test complete, disable fence and come home")
        self.do_fence_disable()
        self.fly_home_land_and_disarm()
