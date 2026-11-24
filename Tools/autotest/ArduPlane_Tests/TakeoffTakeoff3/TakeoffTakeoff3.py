'''
Plane autotest: TakeoffTakeoff3

AP_FLAKE8_CLEAN
'''

import vehicle_test_suite

from test_discovery import TestGroup
from test_discovery import testcase


class TakeoffTakeoff3(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def TakeoffTakeoff3(self):
        '''Test the behaviour of a takeoff in TAKEOFF mode, pt3.'''
        '''
        This is the same as case #1, but with disabled airspeed sensor.

        Conditions:
        - ARSPD_USE=0
        - TKOFF_OPTIONS[0]=0
        - TKOFF_THR_MAX < THR_MAX
        '''

        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
        )
        self.set_parameters({
            "ARSPD_USE": 0.0,
            "THR_MAX": 100.0,
            "TKOFF_DIST": 400.0,
            "TKOFF_LVL_ALT": 30.0,
            "TKOFF_ALT": 100.0,
            "TKOFF_OPTIONS": 0.0,
            "TKOFF_THR_MINACC": 3.0,
            "TKOFF_THR_MAX": 80.0,
            "TECS_PITCH_MAX": 35.0,
            "PTCH_LIM_MAX_DEG": 35.0,
        })
        self.change_mode("TAKEOFF")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Throw the catapult.
        self.set_servo(7, 2000)

        # we expect to maintain this throttle level past the takeoff
        # altitude through to our takeoff altitude:
        expected_takeoff_throttle = 1000+10*self.get_parameter("TKOFF_THR_MAX")

        # Check whether we're at max throttle below TKOFF_LVL_ALT.
        test_alt = self.get_parameter("TKOFF_LVL_ALT")-10
        self.wait_altitude(test_alt, test_alt+2, relative=True)
        w = vehicle_test_suite.WaitAndMaintainServoChannelValue(
            self,
            3,  # throttle
            expected_takeoff_throttle,
            epsilon=10,
            minimum_duration=1,
        )
        w.run()

        # Check whether we're still at max throttle past TKOFF_LVL_ALT.
        test_alt = self.get_parameter("TKOFF_LVL_ALT")+10
        self.wait_altitude(test_alt, test_alt+2, relative=True)

        w = vehicle_test_suite.WaitAndMaintainServoChannelValue(
            self,
            3,  # throttle
            expected_takeoff_throttle,
            epsilon=10,
            minimum_duration=1,
        )
        w.run()

        # Wait for the takeoff to complete.
        target_alt = self.get_parameter("TKOFF_ALT")
        self.wait_altitude(target_alt-2.5, target_alt+2.5, relative=True, minimum_duration=10, timeout=30)

        self.reboot_sitl(force=True)
