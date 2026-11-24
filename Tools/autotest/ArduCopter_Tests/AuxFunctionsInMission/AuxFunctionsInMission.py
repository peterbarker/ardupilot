'''
Copter autotest: AuxFunctionsInMission

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AuxFunctionsInMission(TestGroup):
    @testcase(bucket="CopterTests1d")
    def AuxFunctionsInMission(self):
        '''Test use of auxiliary functions in missions'''
        self.load_mission("aux_functions.txt")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.set_rc(3, 1500)
        self.wait_mode('ALT_HOLD')
        self.change_mode('AUTO')
        self.wait_rtl_complete()
