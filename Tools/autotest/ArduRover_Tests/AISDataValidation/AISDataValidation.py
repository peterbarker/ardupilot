'''
Rover autotest: AISDataValidation

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AISDataValidation(TestGroup):
    @testcase(bucket="Rover")
    def AISDataValidation(self):
        '''Test AIS vessel data validity'''
        self.customise_SITL_commandline([
            "--serial5=sim:AIS",
        ])
        self.set_parameters({
            "SERIAL5_PROTOCOL": 40,
            "SERIAL5_BAUD": 38,
            "AIS_TYPE": 1,
        })
        self.reboot_sitl()

        self.set_parameter("SIM_AIS_COUNT", 3)

        m = self.assert_receive_message('AIS_VESSEL', timeout=60)

        # Validate MMSI
        if m.MMSI == 0:
            raise NotAchievedException("Invalid MMSI: must be non-zero")

        # Validate latitude (-90 to +90 degrees)
        lat_deg = m.lat / 1e7
        if abs(lat_deg) > 90:
            raise NotAchievedException("Invalid latitude: %f" % lat_deg)

        # Validate longitude (-180 to +180 degrees)
        lon_deg = m.lon / 1e7
        if abs(lon_deg) > 180:
            raise NotAchievedException("Invalid longitude: %f" % lon_deg)

        # Validate heading is reasonable (allow up to 65535 for unavailable)
        if m.heading > 65535:
            raise NotAchievedException("Invalid heading: %u" % m.heading)

        # Validate COG is reasonable (allow up to 65535 for unavailable)
        if m.COG > 65535:
            raise NotAchievedException("Invalid COG: %u" % m.COG)

        # Validate velocity (max ~100 knots = 10000 in 0.01 knot units)
        if m.velocity > 10000 and m.velocity != 65535:
            raise NotAchievedException("Invalid velocity: %u" % m.velocity)

        self.progress("AIS data validation passed: MMSI=%u lat=%.6f lon=%.6f" %
                      (m.MMSI, lat_deg, lon_deg))
