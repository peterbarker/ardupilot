'''
Plane autotest: FenceEnableDisableAux

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import WaitModeTimeout

from test_discovery import TestGroup
from test_discovery import testcase


class FenceEnableDisableAux(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def FenceEnableDisableAux(self):
        '''Tests enablement and disablement of fences via aux command'''
        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        enable = 0
        self.set_parameters({
            "FENCE_TYPE": 12,     # Set fence type to polyfence + AltMin
            "FENCE_ALT_MIN": 10,
            "FENCE_ENABLE" : enable,
        })

        if not enable:
            self.progress("Checking fence is not present before being configured")
            m = self.assert_receive_message('SYS_STATUS')
            self.progress("Got (%s)" % str(m))
            if (m.onboard_control_sensors_enabled & fence_bit):
                raise NotAchievedException("Fence enabled before being configured")

        self.load_fence("CMAC-fence.txt")

        self.wait_ready_to_arm()
        # takeoff at a lower altitude to avoid immediately breaching polyfence
        self.takeoff(alt=25)
        self.change_mode("CRUISE")
        self.wait_distance(150, accuracy=20)

        self.run_auxfunc(
            11,
            2,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED
        )

        m = self.assert_receive_message('FENCE_STATUS', timeout=2, verbose=True)

        self.progress("Checking fence is initially OK")
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE,
                               present=True,
                               enabled=True,
                               healthy=True,
                               verbose=False,
                               timeout=30)

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
        # check we are in breach
        self.assert_fence_enabled()
        self.assert_fence_sys_status(True, True, False)

        # wait until we get home
        self.wait_distance_to_home(50, 100, timeout=200)
        # now check we are now not in breach
        self.assert_fence_sys_status(True, True, True)
        # Turn fence off with aux function
        self.run_auxfunc(
            11,
            0,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        # switch back to cruise
        self.change_mode("CRUISE")
        self.wait_distance(150, accuracy=20)

        # re-enable the fences
        self.run_auxfunc(
            11,
            2,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED
        )

        m = self.assert_receive_message('FENCE_STATUS', timeout=2, verbose=True)

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

        # wait to no longer be in breach
        self.wait_distance_to_home(50, 100, timeout=200)
        self.assert_fence_sys_status(True, True, True)

        # fly home and land with fences still enabled
        self.fly_home_land_and_disarm(timeout=250)
        self.do_fence_disable() # Ensure the fence is disabled after test
