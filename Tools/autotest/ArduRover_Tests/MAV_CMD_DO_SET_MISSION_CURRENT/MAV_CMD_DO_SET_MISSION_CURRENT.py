'''
Rover autotest: MAV_CMD_DO_SET_MISSION_CURRENT

AP_FLAKE8_CLEAN
'''

import copy

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_SET_MISSION_CURRENT(TestGroup):
    def MAV_CMD_DO_SET_MISSION_CURRENT_mission(self, target_system=1, target_component=1):
        return copy.copy([
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
        ])

    @testcase(bucket="Rover")
    def MAV_CMD_DO_SET_MISSION_CURRENT(self, target_sysid=None, target_compid=1):
        '''Test handling of CMD_DO_SET_MISSION_CURRENT'''
        if target_sysid is None:
            target_sysid = self.sysid_thismav()
        self.check_mission_upload_download(self.MAV_CMD_DO_SET_MISSION_CURRENT_mission())

        self.set_current_waypoint(2)

        self.set_current_waypoint_using_mav_cmd_do_set_mission_current(2)

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
            p1=17,
            timeout=1,
            target_sysid=target_sysid,
            target_compid=target_compid,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )
