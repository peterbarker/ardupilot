'''
Copter autotest: CommonOriginExternalAHRS

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CommonOriginExternalAHRS(TestGroup):
    @testcase(bucket="CopterTests1a")
    def CommonOriginExternalAHRS(self):
        '''ensure an ExternalAHRS origin is shared into EKF2 and EKF3'''
        # Run an ExternalAHRS (MicroStrain7, which supplies GPS) alongside
        # EKF2 and EKF3.  The ExternalAHRS derives its origin from the true
        # vehicle position and so is unaffected by the SITL GPS glitch, while
        # the EKFs are made slow to get an origin and use the (glitched) SITL
        # GPS.  The ExternalAHRS therefore wins the race to set the common
        # origin; if that origin is correctly shared the EKFs report the
        # un-glitched ExternalAHRS origin rather than their own glitched one.
        self.customise_SITL_commandline([
            "--serial4=sim:MicroStrain7",
        ])
        self.set_parameters({
            'EK2_ENABLE': 1,
            'EK3_ENABLE': 1,
            'EK2_CHECK_SCALE': 1,        # make the EKFs slow to get an origin
            'EK3_CHECK_SCALE': 1,
            'SIM_GPS1_GLTCH_X': 0.001,   # about 100m, only affects the SITL GPS
            'EAHRS_TYPE': 7,             # MicroStrain7
            'SERIAL4_PROTOCOL': 36,      # ExternalAHRS
            'SERIAL4_BAUD': 230400,
            'EAHRS_SENSORS': 0xD,        # GPS|BARO|COMPASS (exclude IMU)
            'AHRS_EKF_TYPE': 11,         # read the ExternalAHRS origin below
        })
        self.reboot_sitl()

        # The ExternalAHRS obtains an origin from its (un-glitched) GPS well
        # before the deliberately-slowed EKFs get one of their own; the
        # common-origin code then pushes that origin into the EKFs.
        self.delay_sim_time(15, reason="ExternalAHRS to get origin and share it")

        # read the ExternalAHRS origin (AHRS_EKF_TYPE==11 selects it)
        ext_origin = self.poll_message('GPS_GLOBAL_ORIGIN')

        # confirm the ExternalAHRS really was the source: its origin matches
        # the true home rather than the ~100m-glitched SITL GPS position
        ext_loc = Location.latlon_only(ext_origin.latitude * 1e-7, ext_origin.longitude * 1e-7)
        dist = self.get_distance(self.sitl_start_location(), ext_loc)
        if dist > 30:
            raise NotAchievedException(
                "ExternalAHRS origin too far from home (%.1fm) - glitched GPS won the race" % dist)

        # each EKF must report the same origin: the common-origin code shared
        # the ExternalAHRS origin into them, so despite their own GPS being
        # glitched they hold the un-glitched ExternalAHRS origin
        for ekf_type, name in (3, "EKF3"), (2, "EKF2"):
            self.set_parameter('AHRS_EKF_TYPE', ekf_type)
            ekf_origin = self.poll_message('GPS_GLOBAL_ORIGIN')
            if (ext_origin.latitude != ekf_origin.latitude or
                    ext_origin.longitude != ekf_origin.longitude or
                    ext_origin.altitude != ekf_origin.altitude):
                raise NotAchievedException("%s did not adopt the ExternalAHRS origin" % name)
