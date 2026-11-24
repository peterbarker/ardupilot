'''
QuadPlane autotest: Q_GUIDED_MODE

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class Q_GUIDED_MODE(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def Q_GUIDED_MODE(self):
        '''test moving in VTOL mode with SET_POSITION_TARGET_GLOBAL_INT'''
        self.set_parameter('Q_GUIDED_MODE', 1)
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=15)
        self.wait_altitude(14, 16, relative=True)

        target_alt = 30
        loc = self.offset_location_ne(self.home_position_as_location(), 50, 50)
        loc.set_alt_m(target_alt, AltFrame.ABOVE_HOME)

        # set position target
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            1,  # reposition flags; 1 means "change to guided"
            0,
            0,
            int(loc.lat * 1e7),
            int(loc.lng * 1e7),
            target_alt,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        )
        self.wait_location(loc, timeout=120, height_accuracy=2)

        self.fly_home_land_and_disarm()
