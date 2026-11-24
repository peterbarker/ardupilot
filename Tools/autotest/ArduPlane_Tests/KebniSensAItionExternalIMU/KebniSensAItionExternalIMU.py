'''
Plane autotest: KebniSensAItionExternalIMU

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class KebniSensAItionExternalIMU(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def KebniSensAItionExternalIMU(self):
        '''Test Kebni SensAItion External IMU-only mode'''
        self.customise_SITL_commandline(["--serial4=sim:SensAItion"])

        self.set_parameters({
            # External AHRS configuration (IMU-only mode at 1000Hz)
            "EAHRS_TYPE": 11,         # SensAItion External AHRS type
            "EAHRS_SENSORS": 14,      # IMU(2) + Baro(4) + Compass(8) = 14
            "EAHRS_OPTIONS": 0,      # Legacy IMU mode
            "SERIAL4_PROTOCOL": 36,   # External AHRS protocol
            "SERIAL4_BAUD": 460800,   # 460800 baud
            "GPS1_TYPE": 1,           # Use SITL GPS for position data
            "AHRS_EKF_TYPE": 3,       # Internal EKF3 (fed by external IMU)
            "EK3_PRIMARY": 0,         # Use external IMU (SensAItion) as primary
            "EK3_OPTIONS": 2,         # Disable EK3 lane switching

            # Ultra high-rate INS filtering
            "INS_GYR_CAL": 1,
        })
        self.reboot_sitl()
        self.delay_sim_time(10, reason="external AHRS to initialise")

        self.progress("Running accelcal")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=4,
            timeout=10,
        )

        # Wait for EKF3 with external IMU to converge
        expected_flags = (mavutil.mavlink.ESTIMATOR_ATTITUDE |
                          mavutil.mavlink.ESTIMATOR_VELOCITY_HORIZ |
                          mavutil.mavlink.ESTIMATOR_VELOCITY_VERT |
                          mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL |
                          mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS |
                          mavutil.mavlink.ESTIMATOR_POS_VERT_ABS)
        self.wait_ekf_flags(expected_flags, 0, timeout=60)

        self.wait_ready_to_arm(timeout=120)
        self.arm_vehicle()
        self.fly_mission("ap1.txt", mission_timeout=120)
        self.disarm_vehicle(force=True)
