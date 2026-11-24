'''
Copter autotest: FenceFloorEnabledLanding

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class FenceFloorEnabledLanding(TestGroup):
    @testcase(bucket="CopterTests2b")
    def FenceFloorEnabledLanding(self):
        """Ensures we can initiate and complete an RTL while the fence is
        enabled.
        """
        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        self.progress("Test Landing while fence floor enabled")
        self.set_parameters({
            "AVOID_ENABLE": 0,
            "FENCE_ENABLE" : 1,
            "FENCE_TYPE": 15,
            "FENCE_ALT_MIN": 20,
            "FENCE_ALT_MAX": 30,
        })

        self.change_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.user_takeoff(alt_min=25)

        # Check fence is enabled
        self.assert_fence_enabled()

        # Change to RC controlled mode
        self.change_mode('LOITER')

        self.set_rc(3, 1800)

        self.wait_mode('RTL', timeout=120)
        # center throttle
        self.set_rc(3, 1500)

        # wait until we are below the fence floor and re-enter loiter
        self.wait_altitude(5, 15, relative=True)
        self.change_mode('LOITER')
        # wait for manual recovery to expire
        self.delay_sim_time(15, reason="manual recovery to expire")

        # lower throttle and try and land
        self.set_rc(3, 1300)
        self.wait_altitude(0, 2, relative=True)
        self.zero_throttle()
        self.wait_landed_and_disarmed()
        self.assert_fence_enabled()
        # must not be in RTL
        self.assert_mode("LOITER")

        # Assert fence is healthy since it was enabled automatically
        self.assert_sensor_state(fence_bit, healthy=True)

        # Disable the fence using mavlink command to ensure cleaned up SITL state
        self.do_fence_disable()
        self.assert_fence_disabled()
