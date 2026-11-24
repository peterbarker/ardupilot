'''
Plane autotest: MAV_CMD_NAV_LOITER_UNLIM

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_LOITER_UNLIM(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def MAV_CMD_NAV_LOITER_UNLIM(self):
        '''test receiving MAV_CMD_NAV_LOITER_UNLIM from GCS'''
        self.start_subtest("refused if LOITER may not be entered from the GCS")
        self.change_mode('FBWA')
        self.set_parameter("FLTMODE_GCSBLOCK", 1 << 10)  # LOITER
        for run_cmd in self.run_cmd, self.run_cmd_int:
            run_cmd(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
            self.assert_mode_is('FBWA')
        self.set_parameter("FLTMODE_GCSBLOCK", 0)

        self.start_subtest("changes into LOITER")
        self.takeoff(10)
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM)
        self.wait_mode('LOITER')
        self.change_mode('GUIDED')
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM)
        self.wait_mode('LOITER')
        self.fly_home_land_and_disarm()
