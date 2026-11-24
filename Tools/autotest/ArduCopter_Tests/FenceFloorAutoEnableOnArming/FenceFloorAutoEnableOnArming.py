'''
Copter autotest: FenceFloorAutoEnableOnArming

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class FenceFloorAutoEnableOnArming(TestGroup):
    @testcase(bucket="CopterTests2d")
    def FenceFloorAutoEnableOnArming(self):
        """Ensures we can auto-enable fences on arming and still takeoff and land"""

        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE

        self.set_parameters({
            "AVOID_ENABLE": 0,
            "FENCE_TYPE": 11,
            "FENCE_ALT_MIN": 10,
            "FENCE_ALT_MAX": 20,
            "FENCE_AUTOENABLE" : 3,
        })

        self.change_mode("GUIDED")
        # Check fence is not enabled
        self.assert_fence_disabled()

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.takeoff(altitude_min=15, mode="GUIDED")

        # Check fence is enabled
        self.assert_fence_enabled()

        # Change to RC controlled mode
        self.set_rc(3, 1800)
        self.change_mode('LOITER')

        self.wait_mode('RTL', timeout=120)
        # Assert fence is not healthy now that we are in RTL
        self.assert_sensor_state(fence_bit, healthy=False)

        self.wait_landed_and_disarmed(0)
        # the breach should have cleared since we auto-disable the
        # fence on landing
        self.assert_fence_disabled()

        # Assert fences have gone now that we have landed and disarmed
        self.assert_sensor_state(fence_bit, present=True, enabled=False)

        # Disable the fence using mavlink command to ensure cleaned up SITL state
        self.assert_fence_disabled()
