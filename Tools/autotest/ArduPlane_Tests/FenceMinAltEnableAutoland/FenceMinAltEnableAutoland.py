'''
Plane autotest: FenceMinAltEnableAutoland

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import WaitModeTimeout

from test_discovery import TestGroup
from test_discovery import testcase


class FenceMinAltEnableAutoland(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def FenceMinAltEnableAutoland(self):
        '''Tests autolanding when alt min fence is enabled'''
        self.set_parameters({
            "FENCE_TYPE": 12,     # Set fence type to min alt and max alt
            "FENCE_ACTION": 1,   # Set action to RTL
            "FENCE_ALT_MIN": 20,
            "FENCE_AUTOENABLE": 0,
            "FENCE_ENABLE" : 1,
            "RTL_AUTOLAND" : 2,
        })

        # Grab Home Position
        self.wait_ready_to_arm()
        self.arm_vehicle()

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
        # switch to FBWA
        self.change_mode("FBWA")
        self.set_rc(3, 1500)    # raise throttle
        self.wait_altitude(25, 35, timeout=50, relative=True)
        self.set_rc(3, 1000)    # lower throttle
        # Now check we can land
        self.fly_home_land_and_disarm()
