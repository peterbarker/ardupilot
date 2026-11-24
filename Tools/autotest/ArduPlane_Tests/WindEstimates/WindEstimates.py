'''
Plane autotest: WindEstimates

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class WindEstimates(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def WindEstimates(self):
        '''fly non-external AHRS, ensure wind estimate correct'''
        self.set_parameters({
            "SIM_WIND_SPD": 5,
            "SIM_WIND_DIR": 45,
        })
        self.wait_ready_to_arm()
        self.takeoff(70)  # default wind sim wind is a sqrt function up to 60m
        self.change_mode('LOITER')
        # use default estimator to determine when to check others:
        self.wait_and_maintain_wind_estimate(
            5, 45, timeout=120, minimum_duration=10)

        for ahrs_type in 0, 2, 3, 10:
            self.start_subtest("Checking AHRS_EKF_TYPE=%u" % ahrs_type)
            self.set_parameter("AHRS_EKF_TYPE", ahrs_type)
            if ahrs_type == 0:
                # DCM's wind triangle assumes the airflow lies along the fuselage, so the
                # angle of attack and turn-direction-dependent sideslip it ignores cost it degrees.
                self.wait_and_maintain_wind_estimate(
                    5, 45,
                    speed_tolerance=1,
                    dir_tolerance=6,
                    minimum_duration=10,
                    timeout=90,
                )
            else:
                self.wait_and_maintain_wind_estimate(
                    5, 45,
                    speed_tolerance=1,
                    minimum_duration=10,
                    timeout=60,
                )
        self.fly_home_land_and_disarm()
