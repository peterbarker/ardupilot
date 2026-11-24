'''
Copter autotest: FenceFloorAutoDisableLanding

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class FenceFloorAutoDisableLanding(TestGroup):
    @testcase(bucket="CopterTests2c")
    def FenceFloorAutoDisableLanding(self):
        """Ensures we can initiate and complete an RTL while the fence is enabled"""

        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        self.progress("Test Landing while fence floor enabled")
        self.set_parameters({
            "AVOID_ENABLE": 0,
            "FENCE_TYPE": 11,
            "FENCE_ALT_MIN": 10,
            "FENCE_ALT_MAX": 20,
            "FENCE_AUTOENABLE" : 1,
        })

        self.change_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.takeoff(altitude_min=15, mode="GUIDED")

        # Check fence is enabled
        self.assert_fence_enabled()

        # Change to RC controlled mode
        self.set_rc(3, 1800)
        self.change_mode('LOITER')

        self.wait_mode('RTL', timeout=120)

        self.wait_landed_and_disarmed(0)
        # the breach should have cleared since we auto-disable the
        # fence on landing
        self.assert_fence_disabled()

        # Assert fences have gone now that we have landed and disarmed
        self.assert_sensor_state(fence_bit, present=True, enabled=False)
