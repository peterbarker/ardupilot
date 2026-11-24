'''
Copter autotest: BattCANReplaceRuntime

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BattCANReplaceRuntime(TestGroup):
    @testcase(bucket="BattCAN")
    def BattCANReplaceRuntime(self):
        '''test CAN battery periphs'''
        self.start_subtest("Replace UAVCAN backend at runtime")
        self.set_parameters({
            "CAN_P1_DRIVER": 1,
            "BATT_MONITOR": 11,  # 4 is analog volt+curr
        })
        self.reboot_sitl()
        self.delay_sim_time(2, reason="CAN battery backend to initialise")
        self.set_parameters({
            "BATT_MONITOR": 8,  # 4 is UAVCAN batterinfo
        })
        self.delay_sim_time(2, reason="battery monitor replacement to take effect")
