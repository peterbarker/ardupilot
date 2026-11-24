'''
Plane autotest: GpsSensorPreArmEAHRS

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class GpsSensorPreArmEAHRS(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def GpsSensorPreArmEAHRS(self):
        '''Test pre-arm checks related to EAHRS_SENSORS using the MicroStrain7 driver'''
        self.customise_SITL_commandline(["--serial4=sim:MicroStrain7"])

        self.set_parameters({
            "EAHRS_TYPE": 7,
            "SERIAL4_PROTOCOL": 36,
            "SERIAL4_BAUD": 230400,
            "GPS1_TYPE": 0, # Disabled (simulate user setup error)
            "GPS2_TYPE": 0, # Disabled (simulate user setup error)
            "AHRS_EKF_TYPE": 11,
            "INS_GYR_CAL": 1,
            "EAHRS_SENSORS": 13, # GPS is enabled
        })
        self.reboot_sitl()
        self.delay_sim_time(5, reason="external AHRS to initialise")
        self.progress("Running accelcal")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=4,
            timeout=5,
        )

        self.assert_prearm_failure("ExternalAHRS: Incorrect number", # Cut short due to message limits.
                                   timeout=30,
                                   other_prearm_failures_fatal=False)

        self.set_parameters({
            "EAHRS_TYPE": 7,
            "SERIAL4_PROTOCOL": 36,
            "SERIAL4_BAUD": 230400,
            "GPS1_TYPE": 1, # Auto
            "GPS2_TYPE": 21, # EAHRS
            "AHRS_EKF_TYPE": 11,
            "INS_GYR_CAL": 1,
            "EAHRS_SENSORS": 13, # GPS is enabled
        })
        self.reboot_sitl()
        self.delay_sim_time(5, reason="external AHRS to initialise")
        self.progress("Running accelcal")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=4,
            timeout=5,
        )
        # Check prearm success with MicroStrain when the first GPS is occupied by another GPS.
        # This supports the use case of comparing MicroStrain dual antenna to another GPS.
        self.wait_ready_to_arm()
