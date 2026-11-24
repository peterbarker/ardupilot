'''
Copter autotest: CorrectedDeltaVelocity

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import WaitAndMaintainLocation

from test_discovery import TestGroup
from test_discovery import testcase


class CorrectedDeltaVelocity(TestGroup):
    @testcase(bucket="CopterTests2d")
    def CorrectedDeltaVelocity(self):
        '''test that AHRS applies accel bias correctly'''
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        takeoff_alt = 10
        target = self.home_position_as_location()
        target.set_alt_m(takeoff_alt, AltFrame.ABOVE_HOME)
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,

            "SIM_ACC1_BIAS_X": 1,
            "SIM_ACC2_BIAS_X": 1,
            "SIM_ACC3_BIAS_X": 1,

            "PLND_ENABLED": 1,
            "SIM_PLD_LAT": target.lat,
            "SIM_PLD_LON": target.lng,
            "SIM_PLD_HEIGHT": 0,
            "SIM_PLD_ALT_LMT": 15,
            "SIM_PLD_DIST_LMT": 10,
            "PLND_TYPE": 1,  # companion
        })
        self.set_analog_rangefinder_parameters()
        self.set_parameter("SIM_SONAR_SCALE", 12)
        self.reboot_sitl()

        self.takeoff(takeoff_alt, mode='LOITER')

        self.run_auxfunc(39, 2)  # enable precision loiter
        WaitAndMaintainLocation(
            self,
            target,
            height_accuracy=5,  # takeoff only promises alt_min-1..alt_min+max_err(=5)
            fn=lambda : self.precision_loiter_to_pos(0, 0, 10, True),
            fn_interval=0.1,
            minimum_duration=10,
        ).run()
        self.run_auxfunc(39, 0)  # disable precision loiter

        # remove the accelerometer bias before landing: left in place,
        # the land detector's filtered earth-frame acceleration sits
        # exactly at LAND_DETECTOR_ACCEL_MAX (both are 1m/s/s), so
        # whether landing is ever detected - and thus whether we ever
        # disarm - hinges on noise:
        self.set_parameters({
            "SIM_ACC1_BIAS_X": 0,
            "SIM_ACC2_BIAS_X": 0,
            "SIM_ACC3_BIAS_X": 0,
        })

        self.change_mode('LAND')
        self.wait_disarmed()
