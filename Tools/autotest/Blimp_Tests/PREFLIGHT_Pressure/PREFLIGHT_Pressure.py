'''
Blimp autotest: PREFLIGHT_Pressure

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class PREFLIGHT_Pressure(TestGroup):
    @testcase(bucket="Blimp")
    def PREFLIGHT_Pressure(self):
        '''test triggering pressure calibration with mavlink command'''
        # as airspeed is not instantiated on Blimp we expect to
        # instantly get back an accepted.
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p3=1, # p3, baro
        )
