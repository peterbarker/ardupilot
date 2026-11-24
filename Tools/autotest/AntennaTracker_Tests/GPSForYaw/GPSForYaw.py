'''
Tracker autotest: GPSForYaw

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSForYaw(TestGroup):
    @testcase(bucket="Tracker")
    def GPSForYaw(self):
        '''Moving baseline GPS yaw'''
        self.load_default_params_file("tracker-gps-for-yaw.parm")
        self.reboot_sitl()

        self.wait_gps_fix_type_gte(6, message_type="GPS2_RAW", verbose=True)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 20:
                break
            m_gps_raw = self.assert_receive_message("GPS2_RAW", verbose=True)
            m_sim = self.assert_receive_message("SIMSTATE", verbose=True)
            gps_raw_hdg = m_gps_raw.yaw * 0.01
            sim_hdg = mavextra.wrap_360(math.degrees(m_sim.yaw))
            if abs(gps_raw_hdg - sim_hdg) > 5:
                raise NotAchievedException("GPS_RAW not tracking simstate yaw")
            self.progress(f"yaw match ({gps_raw_hdg} vs {sim_hdg}")
