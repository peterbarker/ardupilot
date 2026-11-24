'''
Plane autotest: PitotBlockage

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PitotBlockage(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def PitotBlockage(self):
        '''Test detection and isolation of a blocked pitot tube'''
        self.set_parameters({
            "ARSPD_OPTIONS": 15,
            "ARSPD_USE": 1,
            "SIM_WIND_SPD": 7,
            "SIM_WIND_DIR": 0,
            "ARSPD_WIND_MAX": 15,
        })
        self.takeoff(alt=50, mode='TAKEOFF')
        # simulate the effect of a blocked pitot tube
        self.set_parameter("ARSPD_RATIO", 0.1)
        self.wait_parameter_value("ARSPD_USE", 0, timeout=15)
        self.progress("Faulty Sensor Disabled")
        self.delay_sim_time(20, reason="sensor fault monitoring period")
        # simulate the effect of blockage partially clearing
        self.set_parameter("ARSPD_RATIO", 1.0)
        self.delay_sim_time(60, reason="partial blockage monitoring period")
        if (self.get_parameter("ARSPD_USE") == 0):
            self.progress("Faulty Sensor Remains Disabled")
        else:
            raise NotAchievedException("Fault Sensor Re-Enabled")
        # simulate the effect of blockage fully clearing
        self.set_parameter("ARSPD_RATIO", 2.0)
        self.wait_parameter_value("ARSPD_USE", 1, timeout=70)
        self.progress("Sensor Re-Enabled")
        self.fly_home_land_and_disarm()
