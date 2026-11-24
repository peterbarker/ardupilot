'''
Copter autotest: SetModesViaAuxSwitch

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SetModesViaAuxSwitch(TestGroup):
    @testcase(bucket="CopterTests1c")
    def SetModesViaAuxSwitch(self):
        '''"Set modes via auxswitch"'''
        fltmode_ch = int(self.get_parameter("FLTMODE_CH"))
        self.set_rc(fltmode_ch, 1000)
        self.wait_mode("CIRCLE")
        self.set_rc_from_map({9: 1000, 10: 1000})
        self.set_parameters({
            "RC9_OPTION": 18, # land
            "RC10_OPTION": 55, # guided
        })
        self.set_rc(9, 1900)
        self.wait_mode("LAND")
        self.set_rc(10, 1900)
        self.wait_mode("GUIDED")
        self.set_rc(10, 1000) # this re-polls the mode switch
        self.wait_mode("CIRCLE")
