'''
Copter autotest: MissionIndexValidity

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MissionIndexValidity(TestGroup):
    @testcase(bucket="CopterTests2a")
    def MissionIndexValidity(self):
        '''Confirm that attempting to select an invalid mission item is rejected.'''

        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        num_wp = self.get_mission_count()
        accepted_indices = [0, 1, num_wp-1]
        failed_indices = [num_wp]

        for seq in accepted_indices:
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                         p1=seq,
                         timeout=1,
                         want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)

        # param1=-1 means "don't change current item" (reset-only); accepted
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                     p1=-1,
                     timeout=1,
                     want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)

        for seq in failed_indices:
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                         p1=seq,
                         timeout=1,
                         want_result=mavutil.mavlink.MAV_RESULT_FAILED)

        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                     p1=-2,
                     timeout=1,
                     want_result=mavutil.mavlink.MAV_RESULT_DENIED)
