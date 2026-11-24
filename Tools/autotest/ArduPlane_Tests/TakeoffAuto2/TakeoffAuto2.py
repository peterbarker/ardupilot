'''
Plane autotest: TakeoffAuto2

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class TakeoffAuto2(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def TakeoffAuto2(self):
        '''Test the behaviour of an AUTO takeoff, pt2.'''
        '''
        Conditions:
        - ARSPD_USE=0
        - TKOFF_OPTIONS[0]=0
        - TKOFF_THR_MAX > THR_MAX
        '''

        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
        )
        self.set_parameters({
            "ARSPD_USE": 0.0,
            "THR_MAX": 80.0,
            "TKOFF_THR_MAX": 100.0,
            "TKOFF_THR_MINACC": 3.0,
            "TECS_PITCH_MAX": 35.0,
            "PTCH_LIM_MAX_DEG": 35.0,
            "RTL_AUTOLAND": 2, # The mission contains a DO_LAND_START item.
        })

        # Load and start mission. It contains a MAV_CMD_NAV_TAKEOFF item at 100m.
        self.wait_ready_to_arm()

        # Load and start mission. It contains a MAV_CMD_NAV_TAKEOFF item at 100m.
        self.load_mission("catapult.txt", strict=True)
        self.change_mode('AUTO')

        self.arm_vehicle()

        # Throw the catapult.
        self.set_servo(7, 2000)

        # Wait until we're midway through the climb.
        test_alt = 50
        self.wait_altitude(test_alt, test_alt+2, relative=True)

        # Ensure that by then the aircraft still goes at max allowed throttle.
        target_throttle = 1000+10*(self.get_parameter("TKOFF_THR_MAX"))
        self.assert_servo_channel_range(3, target_throttle-10, target_throttle+10)

        # Wait for landing waypoint.
        self.wait_current_waypoint(11, timeout=1200)
        self.wait_disarmed(120)
