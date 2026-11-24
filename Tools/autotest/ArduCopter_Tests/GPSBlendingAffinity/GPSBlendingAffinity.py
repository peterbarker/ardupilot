'''
Copter autotest: GPSBlendingAffinity

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSBlendingAffinity(TestGroup):
    @testcase(bucket="CopterTests1c")
    def GPSBlendingAffinity(self):
        '''test blending when affinity in use'''
        # configure:
        self.set_parameters({
            "WP_YAW_BEHAVIOR": 0,  # do not yaw
            "GPS2_TYPE": 1,
            "SIM_GPS2_TYPE": 1,
            "SIM_GPS2_ENABLE": 1,
            "SIM_GPS1_POS_X": 1.0,
            "SIM_GPS1_POS_Y": -1.0,
            "SIM_GPS2_POS_X": -1.0,
            "SIM_GPS2_POS_Y": 1.0,
            "GPS_AUTO_SWITCH": 2,

            "EK3_AFFINITY" : 1,
            "EK3_IMU_MASK": 7,
            "SIM_IMU_COUNT": 3,
            "INS_ACC3OFFS_X": 0.001,
            "INS_ACC3OFFS_Y": 0.001,
            "INS_ACC3OFFS_Z": 0.001,
        })
        # force-calibration of accel:
        self.run_cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, p5=76)
        self.reboot_sitl()

        alt = 10
        self.takeoff(alt, mode='GUIDED')
        self.fly_guided_move_local(30, 0, alt)
        self.fly_guided_move_local(30, 30, alt)
        self.fly_guided_move_local(0, 30, alt)
        self.fly_guided_move_local(0, 0, alt)
        self.change_mode('LAND')

        current_log_file = self.dfreader_for_current_onboard_log()

        self.wait_disarmed()

        # ensure that the blended solution is always about half-way
        # between the two GPSs:
        current_ts = None
        max_errors = [0, 0, 0]
        while True:
            m = current_log_file.recv_match(type='XKF1')
            if m is None:
                break
            if current_ts is None:
                if m.C != 0:  # noqa
                    continue
                current_ts = m.TimeUS
                measurements = {}
            if m.TimeUS != current_ts:
                current_ts = None
                continue
            measurements[m.C] = (m.PN, m.PE, m.PD)
            if len(measurements) == 3:
                # check lat:
                axis_epsilons = [0.05, 0.05, 0.06]
                for n in 0, 1, 2:
                    expected_blended = 0.5*measurements[0][n] + 0.5*measurements[1][n]
                    epsilon = axis_epsilons[n]
                    error = abs(measurements[2][n] - expected_blended)
                    # self.progress(f"{n=} {error=}")
                    if error > max_errors[n]:
                        max_errors[n] = error
                    if error > epsilon:
                        raise NotAchievedException(f"Blended diverged {n=} {measurements[0][n]=} {measurements[1][n]=} {measurements[2][n]=} {error=}")  # noqa:E501
                current_ts = None
        self.progress(f"{max_errors=}")
