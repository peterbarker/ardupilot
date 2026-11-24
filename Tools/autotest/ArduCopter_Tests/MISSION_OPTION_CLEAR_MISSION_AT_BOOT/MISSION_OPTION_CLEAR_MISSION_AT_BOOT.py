'''
Copter autotest: MISSION_OPTION_CLEAR_MISSION_AT_BOOT

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MISSION_OPTION_CLEAR_MISSION_AT_BOOT(TestGroup):
    @testcase(bucket="CopterTests1a")
    def MISSION_OPTION_CLEAR_MISSION_AT_BOOT(self):
        '''check functionality of mission-clear-at-boot option'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        if len(self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)) != 4:
            raise NotAchievedException("No mission after upload?!")

        self.reboot_sitl()
        if len(self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)) != 4:
            raise NotAchievedException("No mission after reboot?!")

        self.set_parameters({
            "MIS_OPTIONS": 1,
        })
        self.reboot_sitl()

        if len(self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)) != 0:
            raise NotAchievedException("Mission did not clear after reboot with option set")
