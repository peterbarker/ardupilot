'''
Copter autotest: ParameterValidation

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ParameterValidation(TestGroup):
    @testcase(bucket="CopterTests1c")
    def ParameterValidation(self):
        '''Test parameters are checked for validity'''
        # wait 10 seconds for initialisation
        self.delay_sim_time(10, reason="SITL to initialise")
        self.progress("invalid; min must be less than max:")
        self.set_parameters({
            "MOT_PWM_MIN": 100,
            "MOT_PWM_MAX": 50,
        })
        self.drain_mav()
        self.assert_prearm_failure("Motors: Check MOT_PWM_MIN and MOT_PWM_MAX")
        self.progress("invalid; min must be less than max (equal case):")
        self.set_parameters({
            "MOT_PWM_MIN": 100,
            "MOT_PWM_MAX": 100,
        })
        self.drain_mav()
        self.assert_prearm_failure("Motors: Check MOT_PWM_MIN and MOT_PWM_MAX")
        self.progress("Spin min more than 0.3")
        self.set_parameters({
            "MOT_PWM_MIN": 1000,
            "MOT_PWM_MAX": 2000,
            "MOT_SPIN_MIN": 0.5,
        })
        self.drain_mav()
        self.assert_prearm_failure("PreArm: Motors: MOT_SPIN_MIN too high 0.50 > 0.3")
        self.progress("Spin arm more than spin min")
        self.set_parameters({
            "MOT_SPIN_MIN": 0.1,
            "MOT_SPIN_ARM": 0.2,
        })
        self.drain_mav()
        self.assert_prearm_failure("PreArm: Motors: MOT_SPIN_ARM > MOT_SPIN_MIN")
