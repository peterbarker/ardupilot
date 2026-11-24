'''
Plane autotest: FenceMinAltAutoEnableAbort

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import WaitModeTimeout

from test_discovery import TestGroup
from test_discovery import testcase


class FenceMinAltAutoEnableAbort(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def FenceMinAltAutoEnableAbort(self):
        '''Tests autoenablement of the alt min fence and fences on arming'''
        self.set_parameters({
            "FENCE_TYPE": 8,     # Set fence type to min alt
            "FENCE_ACTION": 1,   # Set action to RTL
            "FENCE_ALT_MIN": 25,
            "FENCE_ALT_MAX": 100,
            "FENCE_AUTOENABLE": 3,
            "FENCE_ENABLE" : 0,
            "RTL_AUTOLAND" : 2,
        })

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.takeoff(alt=50, mode='TAKEOFF')
        self.change_mode("FBWA")
        # min alt fence should now be enabled
        self.assert_fence_enabled()
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

        self.load_generic_mission("flaps.txt")
        self.change_mode("AUTO")
        self.wait_distance_to_waypoint(8, 100, 10000000)
        self.set_current_waypoint(8)
        # abort the landing
        self.wait_altitude(10, 20, timeout=200, relative=True)
        self.change_mode("CRUISE")
        self.set_rc(2, 1200)
        # self.set_rc(3, 1600)    # raise throttle
        self.wait_altitude(30, 40, timeout=200, relative=True)
        # min alt fence should now be re-enabled
        self.assert_fence_enabled()

        # re-center pitch stick
        self.set_rc(2, 1500)

        self.change_mode("AUTO")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_ALL)
        self.fly_home_land_and_disarm(timeout=150)
