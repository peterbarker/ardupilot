'''
Rover autotest: MissionFrames

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MissionFrames(TestGroup):
    def test_mission_frame(self, frame, target_system=1, target_component=1):
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                           target_system=target_system,
                           target_component=target_component)
        items = [
            # first item is ignored for missions
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
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
                frame,
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
        ]

        self.check_mission_upload_download(items)

    @testcase(bucket="Rover")
    def MissionFrames(self, target_system=1, target_component=1):
        '''Upload/Download of items in different frames'''
        for frame in (mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                      mavutil.mavlink.MAV_FRAME_GLOBAL):
            self.test_mission_frame(frame,
                                    target_system=1,
                                    target_component=1)
