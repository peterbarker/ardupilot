'''
Copter autotest: BattCANSplitAuxInfo

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BattCANSplitAuxInfo(TestGroup):
    @testcase(bucket="BattCAN")
    def BattCANSplitAuxInfo(self):
        '''test CAN battery periphs'''
        self.start_subtest("Swap UAVCAN backend at runtime")
        self.set_parameters({
            "CAN_P1_DRIVER": 1,
            "BATT_MONITOR": 4,  # 4 is analog volt+curr
            "BATT2_MONITOR": 8,  # 8 is UAVCAN_BatteryInfo
            "BATT_SERIAL_NUM": 0,
            "BATT2_SERIAL_NUM": 0,
            "BATT_OPTIONS": 128,  # allow split auxinfo
            "BATT2_OPTIONS": 128,  # allow split auxinfo
        })
        self.reboot_sitl()
        self.delay_sim_time(2, reason="CAN battery backend to initialise")
        self.set_parameters({
            "BATT_MONITOR": 8,  # 8 is UAVCAN_BatteryInfo
            "BATT2_MONITOR": 4,  # 8 is UAVCAN_BatteryInfo
        })
        self.delay_sim_time(2, reason="battery monitor switch to take effect")
        self.set_parameters({
            "BATT_MONITOR": 4,  # 8 is UAVCAN_BatteryInfo
            "BATT2_MONITOR": 8,  # 8 is UAVCAN_BatteryInfo
        })
        self.delay_sim_time(2, reason="battery monitor switch to take effect")
        self.set_parameters({
            "BATT_MONITOR": 8,  # 8 is UAVCAN_BatteryInfo
            "BATT2_MONITOR": 4,  # 8 is UAVCAN_BatteryInfo
        })
        self.delay_sim_time(2, reason="battery monitor switch to take effect")
