'''
QuadPlane autotest: BatteryFailsafeLandAUTORefused

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryFailsafeLandAUTORefused(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def BatteryFailsafeLandAUTORefused(self):
        '''battery failsafe Land action falls back to RTL if AUTO is refused'''
        self.set_parameters({
            "BATT_MONITOR": 4,
            "BATT_FS_LOW_ACT": 2,  # Land
        })
        self.reboot_sitl()
        # AUTO is refused while waiting for a takeoff in GUIDED if the
        # mission has no takeoff item:
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, 50),
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_LAND_START),
            (mavutil.mavlink.MAV_CMD_NAV_LAND, 10, 0, 0),
        ])
        for rtl_autoland in 1, 2:
            self.start_subtest(f"RTL_AUTOLAND={rtl_autoland}")
            self.set_parameter("RTL_AUTOLAND", rtl_autoland)
            self.change_mode('GUIDED')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.context_push()
            self.context_collect('STATUSTEXT')
            self.set_parameter("BATT_LOW_VOLT", 50)
            self.wait_statustext("Takeoff waypoint required", check_context=True)
            # RTL, entered while waiting for a takeoff, lands in place:
            self.wait_mode('QLAND')
            self.context_pop()
            self.wait_disarmed()
            # the battery failsafe latches until reboot
            self.reboot_sitl()
