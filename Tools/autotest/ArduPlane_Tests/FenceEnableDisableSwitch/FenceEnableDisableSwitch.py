'''
Plane autotest: FenceEnableDisableSwitch

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import WaitModeTimeout

from test_discovery import TestGroup
from test_discovery import testcase


class FenceEnableDisableSwitch(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def FenceEnableDisableSwitch(self):
        '''Tests enablement and disablement of fences on a switch'''
        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        self.set_parameters({
            "FENCE_TYPE": 4,     # Set fence type to polyfence
            "FENCE_ACTION": 6,   # Set action to GUIDED
            "FENCE_ALT_MIN": 10,
            "FENCE_ENABLE" : 0,
            "RC7_OPTION" : 11,   # AC_Fence uses Aux switch functionality
        })

        self.progress("Checking fence is not present before being configured")
        m = self.assert_receive_message('SYS_STATUS')
        self.progress("Got (%s)" % str(m))
        if (m.onboard_control_sensors_enabled & fence_bit):
            raise NotAchievedException("Fence enabled before being configured")

        self.wait_ready_to_arm()
        # takeoff at a lower altitude to avoid immediately breaching polyfence
        self.takeoff(alt=25)
        self.change_mode("FBWA")

        self.load_fence("CMAC-fence.txt")

        self.set_rc_from_map({
            3: 1500,
            7: 2000,
        }) # Turn fence on with aux function

        m = self.assert_receive_message('FENCE_STATUS', timeout=2, verbose=True)

        self.progress("Checking fence is initially OK")
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE,
                               present=True,
                               enabled=True,
                               healthy=True,
                               verbose=False,
                               timeout=30)

        self.progress("Waiting for GUIDED")
        tstart = self.get_sim_time()
        mode = "GUIDED"
        while not self.mode_is(mode, drain_mav=False, poll=False):
            self.mav.messages['HEARTBEAT'].custom_mode
            self.progress("mav.flightmode=%s Want=%s Alt=%f" % (
                self.mav.flightmode, mode, self.get_altitude(relative=True)))
            if (self.get_sim_time_cached() > tstart + 120):
                raise WaitModeTimeout("Did not change mode")
        self.progress("Got mode %s" % mode)
        # check we are in breach
        self.assert_fence_enabled()

        self.set_rc_from_map({
            7: 1000,
        }) # Turn fence off with aux function

        # wait to no longer be in breach
        self.delay_sim_time(5, reason="fence disable to take effect")
        self.assert_fence_disabled()

        self.fly_home_land_and_disarm(timeout=250)
        self.do_fence_disable() # Ensure the fence is disabled after test
