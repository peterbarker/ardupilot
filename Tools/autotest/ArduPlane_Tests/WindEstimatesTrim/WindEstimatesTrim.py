'''
Plane autotest: WindEstimatesTrim

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class WindEstimatesTrim(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def WindEstimatesTrim(self):
        '''DCM wind estimate must be unchanged by a board mounting offset that
        is corrected by AHRS_TRIM, i.e. it must use the trimmed (vehicle)
        fuselage axis, not the raw IMU/board axis.

        In straight-and-level flight the wind triangle's airspeed-fed branch
        projects the measured airspeed along the fuselage axis, so a pitched
        board axis injects a vertical wind of ~airspeed*sin(tilt).  We fly
        straight and level on DCM and compare the steady vertical wind
        estimate with no mount offset against the estimate with an 8-degree
        offset (SIM_BRD_TRIM, a rigid board mounting tilt of both the accel
        and gyro) corrected by AHRS_TRIM.  The trim must not change it.

        We compare the two rather than assert ~zero because the wind triangle
        ignores angle-of-attack, so there is always a small AoA-induced
        vertical wind present in both cases; only the *difference* isolates
        the trim error.
        '''
        def avg_wind(duration=15):
            # average horizontal speed and vertical speed_z of the WIND message
            spd = []
            spd_z = []
            tstart = self.get_sim_time()
            while self.get_sim_time() - tstart < duration:
                m = self.assert_receive_message('WIND', timeout=5)
                spd.append(m.speed)
                spd_z.append(m.speed_z)
            return sum(spd) / len(spd), sum(spd_z) / len(spd_z)

        self.set_parameters({
            "SIM_WIND_SPD": 5,
            "SIM_WIND_DIR": 45,
            "SIM_WIND_DIR_Z": 0,    # purely horizontal truth wind
        })
        self.reboot_sitl()
        # arm and take off with the default estimator (AHRS_EKF_TYPE=0 has no
        # EKF, so arming readiness would time out); switch to DCM in flight:
        self.wait_ready_to_arm()
        self.takeoff(70)  # default wind sim wind is a sqrt function up to 60m
        self.set_parameter("AHRS_EKF_TYPE", 0)  # use DCM
        # straight and level so the airspeed-fed straight-flight branch runs:
        self.change_mode('CRUISE')
        self.delay_sim_time(40, reason="settle into straight level flight and let the DCM wind estimate converge")
        baseline_spd, baseline_z = avg_wind()

        # introduce an 8-degree rigid board pitch mounting offset (accel and
        # gyro rotated together) and correct it with AHRS_TRIM, exactly as a
        # tilted real mount would be handled.  Applied together, the AHRS
        # attitude output is unchanged, so the vehicle keeps flying level:
        trim_rad = math.radians(8)
        self.set_parameters({
            "SIM_BRD_TRIM_Y": trim_rad,
            "AHRS_TRIM_Y": trim_rad,
        })
        self.delay_sim_time(40, reason="let the attitude and wind estimate resettle after introducing the mount offset")
        trimmed_spd, trimmed_z = avg_wind()

        # The board mount offset (corrected by trim) must not move the wind
        # estimate.  The dominant error is vertical: the straight-flight
        # branch projects airspeed along the fuselage axis, so the raw
        # (pitched) board axis shifts speed_z by ~airspeed*sin(8deg).  The
        # horizontal must also be invariant (it was only corrupted when the
        # simulated compass was not rotated with the board, leaving a yaw
        # error; SIM_BRD_TRIM now rotates the compass too).
        self.progress("WIND.speed_z:   baseline=%.2f trimmed=%.2f (d=%.2f)" %
                      (baseline_z, trimmed_z, trimmed_z - baseline_z))
        self.progress("WIND.speed(hz): baseline=%.2f trimmed=%.2f (d=%.2f)" %
                      (baseline_spd, trimmed_spd, trimmed_spd - baseline_spd))
        if abs(trimmed_z - baseline_z) > 1.0:
            raise NotAchievedException(
                "AHRS trim changed the vertical wind estimate: speed_z baseline=%.2f trimmed=%.2f" %
                (baseline_z, trimmed_z))
        if abs(trimmed_spd - baseline_spd) > 1.0:
            raise NotAchievedException(
                "AHRS trim changed the horizontal wind estimate: speed baseline=%.2f trimmed=%.2f" %
                (baseline_spd, trimmed_spd))
        self.fly_home_land_and_disarm()
