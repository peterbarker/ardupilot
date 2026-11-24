'''
Copter autotest: BatteryFailsafeTwoStageSmartRTLDoLandStart

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryFailsafeTwoStageSmartRTLDoLandStart(TestGroup):
    @testcase(bucket="CopterTests2b")
    def BatteryFailsafeTwoStageSmartRTLDoLandStart(self):
        '''Two stage battery failsafe: low SmartRTL then critical Auto DO_LAND_START'''
        self.batt_failsafe_init()
        self.takeoffAndMoveAway()
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 100, 50),                   # 1
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_LAND_START),  # 2
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 20, 30),                    # 3
            (mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 10, 0),                         # 4
        ])
        self.set_parameters({
            'BATT_FS_LOW_ACT': 3,
            'BATT_FS_CRT_ACT': 6,
            'SIM_BATT_VOLTAGE': 11.4,
        })
        self.wait_statustext("Battery 1 is low", timeout=60)
        self.wait_mode("SMART_RTL")
        self.set_parameter('SIM_BATT_VOLTAGE', 10.0)
        self.wait_statustext("Battery 1 is critical", timeout=60)
        self.wait_mode("AUTO_RTL")
        self.wait_current_waypoint(3)
        self.wait_landed_and_disarmed()
        self.reboot_sitl()  # landed away from home
