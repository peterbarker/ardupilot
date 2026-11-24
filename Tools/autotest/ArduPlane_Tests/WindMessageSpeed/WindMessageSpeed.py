'''
Plane autotest: WindMessageSpeed

AP_FLAKE8_CLEAN
'''

import math

from test_discovery import TestGroup
from test_discovery import testcase


class WindMessageSpeed(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def WindMessageSpeed(self):
        '''Test that WIND.speed is horizontal (ground-plane) speed only'''
        # SIM_WIND_DIR_Z is an elevation angle (degrees from horizontal).
        # With dir_z=45 the wind vector is split equally between horizontal
        # and vertical, so the 3D magnitude equals SIM_WIND_SPD while the
        # horizontal magnitude is SIM_WIND_SPD * cos(45) ~= 0.707 * SPD.
        # Before the fix WIND.speed was wind.length() (3D); after the fix it
        # is wind.xy().length() (horizontal only).
        #
        # AHRS_EKF_TYPE=10 (SIM backend) reads the actual 3D SITL wind
        # vector directly.
        wind_spd = 10.0
        wind_dir_z_deg = 45.0
        expected_horizontal = wind_spd * math.cos(math.radians(wind_dir_z_deg))

        self.set_parameters({
            "AHRS_EKF_TYPE": 10,        # SIM backend: returns actual 3D SITL wind
            "SIM_WIND_SPD": wind_spd,
            "SIM_WIND_DIR": 45,
            "SIM_WIND_DIR_Z": wind_dir_z_deg,
        })

        # The SIM AHRS backend returns actual 3D SITL wind, but the
        # fallback logic prevents it being used until in flight.
        self.takeoff(20, mode='TAKEOFF')

        # Without the fix, speed == wind_spd (3D magnitude ~10 m/s).
        # With the fix, speed == horizontal component (~7.07 m/s).
        # A tolerance of 1 m/s distinguishes the two clearly.
        self.wait_message_field_values("WIND", {
            "speed": expected_horizontal,
        }, epsilon=1, timeout=30, minimum_duration=10)

        self.disarm_vehicle(force=True)
        self.reboot_sitl()
