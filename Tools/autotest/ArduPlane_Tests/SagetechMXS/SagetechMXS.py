'''
Plane autotest: SagetechMXS

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location

from test_discovery import TestGroup
from test_discovery import testcase


class SagetechMXS(TestGroup):
    def location_from_ADSB_VEHICLE(self, m) -> Location:
        '''return a Location extracted from an ADSB_VEHICLE mavlink
        message; a geometric ADSB altitude is AMSL'''
        if m.altitude_type != mavutil.mavlink.ADSB_ALTITUDE_TYPE_GEOMETRIC:
            raise ValueError("Expected geometric alt")
        return Location(
            m.lat*1e-7,
            m.lon*1e-7,
            m.altitude/1000.0585,  # mm -> m
            AltFrame.ABSOLUTE,
        )

    @testcase(bucket="PlaneTests1a")
    def SagetechMXS(self):
        '''test Sagetech MXS ADSB device driver'''
        sim_name = "sagetech_mxs"
        self.set_parameters({
            "SERIAL5_PROTOCOL": 35,
            "ADSB_TYPE": 4,  # Sagetech-MXS
            "SIM_ADSB_TYPES": 8,  # Sagetech-MXS
            "SIM_ADSB_COUNT": 5,
        })
        self.customise_SITL_commandline(["--serial5=sim:%s" % sim_name])
        m = self.assert_receive_message("ADSB_VEHICLE")
        adsb_vehicle_loc = self.location_from_ADSB_VEHICLE(m)
        self.progress("ADSB Vehicle at loc %s" % str(adsb_vehicle_loc))
        home = self.home_position_as_location()
        self.assert_distance(home, adsb_vehicle_loc, 0, 10000)
