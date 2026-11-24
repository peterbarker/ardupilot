'''
Copter autotest: RTLSpeed

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class RTLSpeed(TestGroup):
    @testcase(bucket="CopterTests1e")
    def RTLSpeed(self):
        """Test RTL Speed parameters"""
        rtl_speed_ms = 7
        wpnav_speed_ms = 4
        wpnav_accel_mss = 3
        tolerance = 0.5
        self.load_mission("copter_rtl_speed.txt")
        self.set_parameters({
            'WP_ACC': wpnav_accel_mss,
            'RTL_SPEED_MS': rtl_speed_ms,
            'WP_SPD': wpnav_speed_ms,
        })
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.set_rc(3, 1600)
        self.wait_altitude(19, 25, relative=True)
        self.wait_groundspeed(
            wpnav_speed_ms-tolerance,
            wpnav_speed_ms+tolerance,
            minimum_duration=20,
            timeout=30,
        )
        self.change_mode('RTL')
        self.wait_groundspeed(
            rtl_speed_ms-tolerance,
            rtl_speed_ms+tolerance,
            minimum_duration=5,
        )
        self.change_mode('AUTO')
        # we are returning on the same path, so should see a zero velocity:
        self.wait_groundspeed(
            0-tolerance,
            0+tolerance,
        )
        self.wait_groundspeed(
            wpnav_speed_ms-tolerance,
            wpnav_speed_ms+tolerance,
            minimum_duration=5,
        )
        self.do_RTL()
