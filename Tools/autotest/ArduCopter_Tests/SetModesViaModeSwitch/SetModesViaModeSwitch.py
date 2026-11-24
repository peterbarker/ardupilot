'''
Copter autotest: SetModesViaModeSwitch

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SetModesViaModeSwitch(TestGroup):
    @testcase(bucket="CopterTests1a")
    def SetModesViaModeSwitch(self):
        '''Set modes via modeswitch'''
        fltmode_ch = 5
        self.set_parameter("FLTMODE_CH", fltmode_ch)
        self.set_rc(fltmode_ch, 1000) # PWM for mode1
        testmodes = [("FLTMODE1", 4, "GUIDED", 1165),
                     ("FLTMODE2", 2, "ALT_HOLD", 1295),
                     ("FLTMODE3", 6, "RTL", 1425),
                     ("FLTMODE4", 7, "CIRCLE", 1555),
                     ("FLTMODE5", 1, "ACRO", 1685),
                     ("FLTMODE6", 17, "BRAKE", 1815),
                     ]
        for mode in testmodes:
            (parm, parm_value, name, pwm) = mode
            self.set_parameter(parm, parm_value)

        for mode in reversed(testmodes):
            (parm, parm_value, name, pwm) = mode
            self.set_rc(fltmode_ch, pwm)
            self.wait_mode(name)

        for mode in testmodes:
            (parm, parm_value, name, pwm) = mode
            self.set_rc(fltmode_ch, pwm)
            self.wait_mode(name)

        for mode in reversed(testmodes):
            (parm, parm_value, name, pwm) = mode
            self.set_rc(fltmode_ch, pwm)
            self.wait_mode(name)
