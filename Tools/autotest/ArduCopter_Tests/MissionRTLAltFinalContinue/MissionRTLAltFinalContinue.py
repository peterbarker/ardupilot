'''
Copter autotest: MissionRTLAltFinalContinue

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MissionRTLAltFinalContinue(TestGroup):
    @testcase(bucket="CopterTests2b")
    def MissionRTLAltFinalContinue(self):
        '''ensure AUTO mission can continue past a NAV_RETURN_TO_LAUNCH item
        even if the vehicle holds altitude at RTL_ALT_FINAL_M'''
        target_alt = 5
        self.set_parameters({
            "AUTO_OPTIONS": 3,
            "RTL_ALT_FINAL_M": target_alt,
        })
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0),
        ])

        # ensure the vehicle reaches the LAND command instead of getting stuck at RTL
        self.wait_current_waypoint(4, timeout=60)
        self.wait_disarmed()
