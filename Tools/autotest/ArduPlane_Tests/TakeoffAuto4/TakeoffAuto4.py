'''
Plane autotest: TakeoffAuto4

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class TakeoffAuto4(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def TakeoffAuto4(self):
        '''Test the behaviour of an AUTO takeoff, pt4.'''
        '''
        Conditions:
        - ARSPD_USE=0
        - TKOFF_OPTIONS[0]=1
        '''

        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
        )
        self.set_parameters({
            "ARSPD_USE": 0.0,
            "THR_MAX": 80.0,
            "THR_MIN": 0.0,
            "TKOFF_OPTIONS": 1.0,
            "TKOFF_THR_MAX": 100.0,
            "TKOFF_THR_MINACC": 3.0,
            "TECS_PITCH_MAX": 35.0,
            "TKOFF_THR_MAX_T": 3.0,
            "PTCH_LIM_MAX_DEG": 35.0,
            "RTL_AUTOLAND": 2, # The mission contains a DO_LAND_START item.
        })

        self.wait_ready_to_arm()

        # Load and start mission. It contains a MAV_CMD_NAV_TAKEOFF item at 100m.
        self.load_mission("catapult.txt", strict=True)
        self.change_mode('AUTO')

        self.arm_vehicle()

        # Throw the catapult.
        self.set_servo(7, 2000)

        # Ensure that TKOFF_THR_MAX_T is respected.
        self.delay_sim_time(self.get_parameter("TKOFF_THR_MAX_T")-1, reason="takeoff max throttle period")
        target_throttle = 1000+10*(self.get_parameter("TKOFF_THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Ensure that after that the aircraft still goes to maximum throttle.
        test_alt = 50
        self.wait_altitude(test_alt, test_alt+2, relative=True)
        target_throttle = 1000+10*(self.get_parameter("TKOFF_THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Wait for landing waypoint.
        self.wait_current_waypoint(11, timeout=1200)
        self.wait_disarmed(120)
