'''
Plane autotest: GliderPullup

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GliderPullup(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def GliderPullup(self):
        '''test pullup of glider after ALTITUDE_WAIT'''
        self.start_subtest("test glider pullup")

        self.customise_SITL_commandline(
            [],
            model="glider",
            wipe=True)

        self.set_parameter('LOG_DISARMED', 1)

        self.set_parameters({
            "PUP_ENABLE": 1,
            "SERVO6_FUNCTION": 0, # balloon lift
            "SERVO10_FUNCTION": 156, # lift release
            "EK3_IMU_MASK": 1, # lane switches just make log harder to read
            "AHRS_OPTIONS": 4, # don't disable airspeed based on EKF checks
            "ARSPD_OPTIONS": 0, # don't disable airspeed
            "ARSPD_WIND_GATE": 0,
        })

        self.set_servo(6, 1000)

        self.start_flying_mission("glider-pullup-mission.txt")
        self.context_collect('STATUSTEXT')

        self.progress("Start balloon lift")
        self.set_servo(6, 2000)

        self.wait_text("Reached altitude", check_context=True, timeout=1000)
        self.wait_text("Start pullup airspeed", check_context=True)
        self.wait_text("Pullup airspeed", check_context=True)
        self.wait_text("Pullup pitch", check_context=True)
        self.wait_text("Pullup level", check_context=True)
        self.wait_text("Loiter to alt complete", check_context=True, timeout=1000)
        self.wait_text("Flare", check_context=True, timeout=400)
        self.wait_text("Auto disarmed", check_context=True, timeout=200)
