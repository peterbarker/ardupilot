'''
Plane autotest: TakeoffIdleThrottle

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class TakeoffIdleThrottle(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def TakeoffIdleThrottle(self):
        '''Apply idle throttle before takeoff.'''
        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
        )
        self.set_parameters({
            "TKOFF_THR_IDLE": 20.0,
            "TKOFF_THR_MINSPD": 3.0,
        })
        self.change_mode("TAKEOFF")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Ensure that the throttle rises to idle throttle.
        expected_idle_throttle = 1000+10*self.get_parameter("TKOFF_THR_IDLE")
        self.assert_servo_channel_range(3, expected_idle_throttle-10, expected_idle_throttle+10)

        # Launch the catapult
        self.set_servo(6, 1000)

        self.delay_sim_time(5, reason="catapult launch to establish flight")
        self.change_mode('RTL')

        self.fly_home_land_and_disarm()
