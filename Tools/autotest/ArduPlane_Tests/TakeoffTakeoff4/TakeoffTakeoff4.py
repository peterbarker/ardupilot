'''
Plane autotest: TakeoffTakeoff4

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class TakeoffTakeoff4(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def TakeoffTakeoff4(self):
        '''Test the behaviour of a takeoff in TAKEOFF mode, pt4.'''
        '''
        This is the same as case #3, but with almost stock parameters and without a catapult.

        Conditions:
        - ARSPD_USE=0
        '''

        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
        )
        self.set_parameters({
            "ARSPD_USE": 0.0,
            "TKOFF_LVL_ALT": 30,
        })
        self.change_mode("TAKEOFF")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Check whether we're at max throttle below TKOFF_LVL_ALT.
        test_alt = self.get_parameter("TKOFF_LVL_ALT")-10
        self.wait_altitude(test_alt, test_alt+2, relative=True)
        target_throttle = 1000+10*(self.get_parameter("THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Check whether we're still at max throttle past TKOFF_LVL_ALT.
        test_alt = self.get_parameter("TKOFF_LVL_ALT")+10
        self.wait_altitude(test_alt, test_alt+2, relative=True)
        target_throttle = 1000+10*(self.get_parameter("THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Wait for the takeoff to complete.
        target_alt = self.get_parameter("TKOFF_ALT")
        self.wait_altitude(target_alt-5, target_alt, relative=True)

        # Wait a bit for the Takeoff altitude to settle.
        self.delay_sim_time(5, reason="takeoff altitude to settle")

        self.fly_home_land_and_disarm()
