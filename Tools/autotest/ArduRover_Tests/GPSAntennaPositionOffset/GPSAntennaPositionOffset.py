'''
Rover autotest: GPSAntennaPositionOffset

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSAntennaPositionOffset(TestGroup):
    @testcase(bucket="Rover")
    def GPSAntennaPositionOffset(self):
        '''test GPS antenna position offset is applied'''
        [SIM_GPS1_POS_X, SIM_GPS1_POS_Y, SIM_GPS1_POS_Z] = [1.0, 2.0, 3.0]
        self.set_parameters({
            "SIM_GPS1_POS_X": SIM_GPS1_POS_X,
            "SIM_GPS1_POS_Y": SIM_GPS1_POS_Y,
            "SIM_GPS1_POS_Z": SIM_GPS1_POS_Z,
        })
        self.wait_ready_to_arm()
        gps_m = self.assert_receive_message("GPS_RAW_INT")
        lat = math.degrees(math.radians(gps_m.lat)*1.0e-7)
        lon = math.degrees(math.radians(gps_m.lon)*1.0e-7)
        alt = gps_m.alt/1000.0
        self.set_parameters({
            "SIM_GPS1_POS_X": 0.0,
            "SIM_GPS1_POS_Y": 0.0,
            "SIM_GPS1_POS_Z": 0.0,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        gps_m = self.assert_receive_message("GPS_RAW_INT")

        self.start_subsubtest("Check GPS position changes for altitude")
        alt_with_offset = alt + SIM_GPS1_POS_Z
        if abs(gps_m.alt/1000.0 - alt_with_offset) > 1:
            raise NotAchievedException(f"Unexpected GPS altitude (want {alt_with_offset}, got {gps_m.alt})")

        self.start_subsubtest("Check GPS position changes for latitude and longitude")
        Horizontaldistance = mavextra.distance_from(gps_m, lat, lon)
        expected_distance = math.sqrt(SIM_GPS1_POS_X**2 + SIM_GPS1_POS_Y**2)
        if abs(Horizontaldistance - expected_distance) > 1:
            raise NotAchievedException(f"Unexpected GPS position (want {expected_distance}, got {Horizontaldistance})")
