'''
Plane autotest: DO_SET_HOME_in_RTL

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class DO_SET_HOME_in_RTL(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def DO_SET_HOME_in_RTL(self):
        '''check RTL follows a home position which is changed while in RTL'''
        self.takeoff(50, mode='TAKEOFF')
        self.change_mode('RTL')

        self.start_subtest("set home to a location")
        home = self.home_position_as_location()
        new_home = self.offset_location_ne(home, 1000, 0)
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            p5=int(new_home.lat * 1e7),
            p6=int(new_home.lng * 1e7),
            p7=home.get_alt_m(AltFrame.ABSOLUTE),
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
        )
        self.wait_distance_to_location(new_home, 0, 200, timeout=120)

        self.start_subtest("set home to the current location")
        self.change_mode('GUIDED')
        away = self.offset_location_ne(new_home, 0, 800)
        away.set_alt_m(50, AltFrame.ABOVE_HOME)
        self.send_do_reposition(away)
        self.wait_distance_to_location(new_home, 600, 10000, timeout=120)
        self.change_mode('RTL')
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_HOME, p1=1)
        here = self.get_location()
        # without RTL picking up the new home we would fly back to new_home:
        self.wait_distance_to_location(here, 0, 300, minimum_duration=30, timeout=60)

        # home is locked; reboot to release it
        self.reboot_sitl(force=True)
