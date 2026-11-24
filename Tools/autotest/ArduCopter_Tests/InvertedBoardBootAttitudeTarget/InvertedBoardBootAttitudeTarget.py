'''
Copter autotest: InvertedBoardBootAttitudeTarget

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class InvertedBoardBootAttitudeTarget(TestGroup):
    @testcase(bucket="CopterTests2b")
    def InvertedBoardBootAttitudeTarget(self):
        '''check a board mounted upside-down does not get an inverted attitude target at boot'''
        # A vehicle with AHRS_ORIENTATION=8 and EK3_PRIMARY=2 flipped
        # on takeoff.  The boot gyro calibration left an accel sample
        # without the board rotation applied, DCM aligned to it and so
        # started inverted, and when the AHRS switched from DCM to
        # EKF3 the attitude controller's inertial frame reset carried
        # that 180 degree error into the attitude target.
        self.set_parameters({
            "AHRS_ORIENTATION": 8,  # ROTATION_ROLL_180
            "SIM_IMU_ORIENT": 8,
            "SIM_IMU_COUNT": 3,
            "EK3_IMU_MASK": 7,
            "EK3_PRIMARY": 2,
        })
        self.reboot_sitl()
        self.wait_ekf_happy(require_absolute=False)

        dfreader = self.dfreader_for_current_onboard_log()
        max_tilt = 10  # degrees; the vehicle is sitting level on the ground
        ekf_active_us = None
        first_att_us = None
        while True:
            m = dfreader.recv_match(type=['ATT', 'AHR2', 'MSG'])
            if m is None:
                break
            mtype = m.get_type()
            if mtype == 'MSG':
                if ekf_active_us is None and "AHRS: EKF3 active" in m.Message:
                    ekf_active_us = m.TimeUS
                continue
            if mtype == 'ATT':
                if first_att_us is None:
                    first_att_us = m.TimeUS
                fields = ['Roll', 'Pitch', 'DesRoll', 'DesPitch']
            else:
                # DCM attitude
                fields = ['Roll', 'Pitch']
            for field in fields:
                value = getattr(m, field)
                if abs(value) > max_tilt:
                    raise NotAchievedException(
                        f"{mtype}.{field}={value:.1f} at {m.TimeUS*1e-6:.2f}s; vehicle is level")

        if ekf_active_us is None:
            raise NotAchievedException("Did not find AHRS switch to EKF3 in log")
        if first_att_us is None or first_att_us > ekf_active_us:
            raise NotAchievedException("No attitude logged before the AHRS switched to EKF3")
