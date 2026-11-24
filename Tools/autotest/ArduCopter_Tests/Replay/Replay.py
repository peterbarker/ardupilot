'''
Copter autotest: Replay

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class Replay(TestGroup):
    def test_replay_gps_bit(self):
        self.set_parameters({
            "LOG_REPLAY": 1,
            "LOG_DISARMED": 1,
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 1,
            "AHRS_TRIM_X": 0.01,
            "AHRS_TRIM_Y": -0.03,
            "GPS2_TYPE": 1,
            "GPS1_POS_X": 0.1,
            "GPS1_POS_Y": 0.2,
            "GPS1_POS_Z": 0.3,
            "GPS2_POS_X": -0.1,
            "GPS2_POS_Y": -0.02,
            "GPS2_POS_Z": -0.31,
            "INS_POS1_X": 0.12,
            "INS_POS1_Y": 0.14,
            "INS_POS1_Z": -0.02,
            "INS_POS2_X": 0.07,
            "INS_POS2_Y": 0.012,
            "INS_POS2_Z": -0.06,
            "RNGFND1_TYPE": 1,
            "RNGFND1_PIN": 0,
            "RNGFND1_SCALING": 30,
            "RNGFND1_POS_X": 0.17,
            "RNGFND1_POS_Y": -0.07,
            "RNGFND1_POS_Z": -0.005,
            "SIM_SONAR_SCALE": 30,
            "SIM_GPS2_ENABLE": 1,
        })
        self.reboot_sitl()

        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_LOGGING, True, True, True)

        current_log_filepath = self.current_onboard_log_filepath()
        self.progress("Current log path: %s" % str(current_log_filepath))

        self.change_mode("LOITER")
        self.wait_ready_to_arm(require_absolute=True)
        self.arm_vehicle()
        self.takeoffAndMoveAway()
        self.do_RTL()

        self.reboot_sitl()

        return current_log_filepath

    def test_replay_trim_bit(self):
        '''with significant AHRS_TRIM set, replayed estimators must
        reproduce the trim-dependent outputs of the original flight'''
        self.set_parameters({
            "LOG_REPLAY": 1,
            "LOG_DISARMED": 1,
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 1,
            "AHRS_TRIM_X": math.radians(3),
            "AHRS_TRIM_Y": math.radians(-2),
        })
        self.reboot_sitl()

        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_LOGGING, True, True, True)

        current_log_filepath = self.current_onboard_log_filepath()
        self.progress("Current log path: %s" % str(current_log_filepath))

        self.change_mode("LOITER")
        self.wait_ready_to_arm(require_absolute=True)
        self.arm_vehicle()
        self.takeoffAndMoveAway()
        self.do_RTL()

        self.reboot_sitl()

        return current_log_filepath

    def test_replay_gps_yaw_bit(self):
        self.load_default_params_file("copter-gps-for-yaw.parm")
        self.set_parameters({
            "LOG_REPLAY": 1,
            "LOG_DISARMED": 1,
        })
        self.reboot_sitl()

        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_LOGGING, True, True, True)
        self.wait_gps_fix_type_gte(6, message_type="GPS2_RAW", verbose=True)

        current_log_filepath = self.current_onboard_log_filepath()
        self.progress("Current log path: %s" % str(current_log_filepath))

        self.change_mode("LOITER")
        self.wait_ready_to_arm(require_absolute=True)
        self.arm_vehicle()
        self.takeoffAndMoveAway()
        self.do_RTL()

        self.reboot_sitl()

        return current_log_filepath

    def test_replay_beacon_bit(self):
        self.set_parameters({
            "LOG_REPLAY": 1,
            "LOG_DISARMED": 1,
        })

        old_onboard_logs = sorted(self.log_list())
        self.BeaconPosition()
        new_onboard_logs = sorted(self.log_list())

        self.reboot_sitl()

        log_difference = [x for x in new_onboard_logs if x not in old_onboard_logs]
        return log_difference[1] # index depends on the reboots and ordering thereof in BeaconPosition!

    def test_replay_optical_flow_bit(self):
        self.set_parameters({
            "LOG_REPLAY": 1,
            "LOG_DISARMED": 1,
        })

        old_onboard_logs = sorted(self.log_list())
        self.OpticalFlowLimits()
        new_onboard_logs = sorted(self.log_list())

        log_difference = [x for x in new_onboard_logs if x not in old_onboard_logs]
        print("log difference: %s" % str(log_difference))
        return log_difference[0]

    def test_replay_body_odom_bit(self):
        self.set_parameters({
            "LOG_REPLAY": 1,
            "LOG_DISARMED": 1,

            "EK3_SRC1_POSXY": 6,
            "EK3_SRC1_VELXY": 6,
            "EK3_SRC1_POSZ": 6,
            "EK3_SRC1_VELZ": 6,

            "SIM_VICON_TMASK": 8, # send VISION_POSITION_DELTA for FuseBodyOdom

            "VISO_TYPE": 2,
            "SERIAL5_PROTOCOL": 2,
            "ARMING_SKIPCHK": (1 << 18), # suppress yaw misalignment complaint
        })

        self.customise_SITL_commandline(["--serial5=sim:vicon:"]) # implicit reboot

        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_LOGGING, True, True, True)

        current_log_filepath = self.current_onboard_log_filepath()
        self.progress("Current log path: %s" % str(current_log_filepath))

        self.change_mode("LOITER")
        self.wait_ready_to_arm(require_absolute=False)
        # require_absolute=False does not wait for the EKF origin (and thus home)
        # to be set. Block until home is set so the GUIDED takeoff's ABOVE_HOME
        # alt frame conversion succeeds.
        self.poll_home_position()
        self.arm_vehicle()
        self.takeoffAndMoveAway()
        self.do_RTL()

        self.customise_SITL_commandline([]) # implicit reboot

        return current_log_filepath

    @testcase(bucket="CopterTests2b")
    def Replay(self):
        '''test replay correctness'''
        self.progress("Building Replay")
        self.build_replay()
        self.set_parameters({
            "LOG_DARM_RATEMAX": 0,
            "LOG_FILE_RATEMAX": 0,
            "LOG_FILE_BUFSIZE": 32767,
        })

        bits = [
            ('GPS', self.test_replay_gps_bit),
            ('Trim', self.test_replay_trim_bit),
            ('GPSForYaw', self.test_replay_gps_yaw_bit),
            ('WindAndAirspeed', self.test_replay_wind_and_airspeed_bit),
            ('BodyOdom', self.test_replay_body_odom_bit),
            ('Beacon', self.test_replay_beacon_bit),
            ('OpticalFlow', self.test_replay_optical_flow_bit),
        ]
        for (name, func) in bits:
            self.start_subtest("%s" % name)
            self.test_replay_bit(func)
