'''
Copter autotest: UBloxConfigWhileArmed

AP_FLAKE8_CLEAN
'''

import os

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class UBloxConfigWhileArmed(TestGroup):
    @testcase(bucket="CopterTests2d")
    def UBloxConfigWhileArmed(self):
        '''regression: simulated M10 saves must remain available while armed'''
        self.context_collect('STATUSTEXT')
        try:
            self.set_parameters({
                "GPS_AUTO_CONFIG": 1,
                "GPS_SAVE_CFG": 2,
                "GPS1_GNSS_MODE": 35,  # GPS, SBAS and QZSS
                "SIM_GPS1_UBXMDL": self.UBX_MODEL_M10_SPG510,
                "DISARM_DELAY": 0,
            })
            self.ublox_sim_reset_saved_config()
            self.ublox_sim_boot()
            filepath = self.ublox_sim_config_filepath()

            self.wait_ready_to_arm()
            self.arm_vehicle()
            saved_mtime = os.stat(filepath).st_mtime_ns
            self.set_parameter("GPS1_GNSS_MODE", 33)  # disable SBAS; sends a BBR VALSET
            start = self.get_sim_time()
            while self.get_sim_time_cached() - start < 60:
                self.delay_sim_time(1, reason="wait for armed M10 configuration save")
                if os.stat(filepath).st_mtime_ns != saved_mtime:
                    break
            else:
                raise NotAchievedException("M10 configuration was not saved while armed")
            if not self.armed():
                raise NotAchievedException("Vehicle disarmed during M10 configuration save")
            self.disarm_vehicle()
        finally:
            self.ublox_sim_reset_saved_config()
