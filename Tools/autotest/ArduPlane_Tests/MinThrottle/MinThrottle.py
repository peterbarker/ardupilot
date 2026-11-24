'''
Plane autotest: MinThrottle

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MinThrottle(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def MinThrottle(self):
        '''Make sure min throttle does not apply in manual mode and does in FBWA'''

        servo_min = self.get_parameter("RC3_MIN")
        servo_max = self.get_parameter("RC3_MAX")
        min_throttle = 10
        servo_min_throttle = servo_min + (servo_max - servo_min) * (min_throttle / 100)

        # Set min throttle
        self.set_parameter('THR_MIN', min_throttle)

        # Should be 0 throttle while disarmed
        self.change_mode("MANUAL")
        self.drain_mav() # make sure we have the latest data before checking throttle output
        self.assert_servo_channel_value(3, servo_min)

        # Arm and check throttle is still 0 in manual
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.drain_mav()
        self.assert_servo_channel_value(3, servo_min)

        # FBWA should apply throttle min
        self.change_mode("FBWA")
        self.drain_mav()
        self.assert_servo_channel_value(3, servo_min_throttle)

        # But not when disarmed
        self.disarm_vehicle()
        self.drain_mav()
        self.assert_servo_channel_value(3, servo_min)
