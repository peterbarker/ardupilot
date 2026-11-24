'''
Plane autotest: FenceMinAltAutoEnable

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import WaitModeTimeout

from test_discovery import TestGroup
from test_discovery import testcase


class FenceMinAltAutoEnable(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def FenceMinAltAutoEnable(self):
        '''Tests autoenablement of the alt min fence and fences on arming'''
        self.set_parameters({
            "FENCE_TYPE": 9,     # Set fence type to min alt and max alt
            "FENCE_ACTION": 1,   # Set action to RTL
            "FENCE_ALT_MIN": 25,
            "FENCE_ALT_MAX": 100,
            "FENCE_AUTOENABLE": 3,
            "FENCE_ENABLE" : 0,
            "RTL_AUTOLAND" : 2,
        })

        # check we can takeoff again
        for i in [1, 2]:
            # Grab Home Position
            self.wait_ready_to_arm()
            self.arm_vehicle()
            # max alt fence should now be enabled
            if i == 1:
                self.assert_fence_enabled()

            self.takeoff(alt=50, mode='TAKEOFF')
            self.change_mode("FBWA")
            self.set_rc(3, 1100)    # lower throttle

            self.progress("Waiting for RTL")
            tstart = self.get_sim_time()
            mode = "RTL"
            while not self.mode_is(mode, drain_mav=False, poll=False):
                self.mav.messages['HEARTBEAT'].custom_mode
                self.progress("mav.flightmode=%s Want=%s Alt=%f" % (
                    self.mav.flightmode, mode, self.get_altitude(relative=True)))
                if (self.get_sim_time_cached() > tstart + 120):
                    raise WaitModeTimeout("Did not change mode")
            self.progress("Got mode %s" % mode)
            self.fly_home_land_and_disarm()
            self.change_mode("FBWA")
            self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_ALL)
            self.set_current_waypoint(0, check_afterwards=False)
            self.set_rc(3, 1000)    # lower throttle
