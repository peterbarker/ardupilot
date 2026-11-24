'''
Plane autotest: MissionJumpTags

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MissionJumpTags(TestGroup):
    def mission_home_point(self, target_system=1, target_component=1):
        '''just an empty-ish item-int to store home'''
        return self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, # current
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            0, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    def mission_jump_tag(self, tag, target_system=1, target_component=1):
        '''create a jump tag mission item'''
        return self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_JUMP_TAG,
            0, # current
            0, # autocontinue
            tag, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            0, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    def mission_do_jump_tag(self, tag, target_system=1, target_component=1):
        '''create a jump tag mission item'''
        return self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_DO_JUMP_TAG,
            0, # current
            0, # autocontinue
            tag, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            0, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    def mission_anonymous_waypoint(self, target_system=1, target_component=1):
        '''just a boring waypoint'''
        return self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, # current
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            1, # latitude
            1, # longitude
            1, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    def MissionJumpTags_missing_jump_target(self, target_system=1, target_component=1):
        self.start_subtest("Check missing-jump-tag behaviour")
        jump_target = 2
        mission = [
            self.mission_home_point(),
            self.mission_anonymous_waypoint(),
            self.mission_anonymous_waypoint(),
            self.mission_jump_tag(jump_target),
            self.mission_anonymous_waypoint(),
            self.mission_anonymous_waypoint(),
        ]
        self.renumber_mission_items(mission)
        self.check_mission_upload_download(mission)
        self.progress("Checking incorrect tag behaviour")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_JUMP_TAG,
            p1=jump_target + 1,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED
        )
        self.progress("Checking correct tag behaviour")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_JUMP_TAG,
            p1=jump_target,
        )
        self.assert_current_waypoint(4)

    def MissionJumpTags_do_jump_to_bad_tag(self, target_system=1, target_component=1):
        mission = [
            self.mission_home_point(),
            self.mission_anonymous_waypoint(),
            self.mission_do_jump_tag(17),
            self.mission_anonymous_waypoint(),
        ]
        self.renumber_mission_items(mission)
        self.check_mission_upload_download(mission)
        self.change_mode('AUTO')
        self.arm_vehicle()
        self.set_current_waypoint(2, check_afterwards=False)
        self.assert_mode('RTL')
        self.disarm_vehicle()

    def MissionJumpTags_jump_tag_at_end_of_mission(self, target_system=1, target_component=1):
        mission = [
            self.mission_home_point(),
            self.mission_anonymous_waypoint(),
            self.mission_jump_tag(17),
        ]
        # Jumping to an end of a mission, either DO_JUMP or DO_JUMP_TAG will result in a failed attempt.
        # The failure is from mission::set_current_cmd() returning false if it can not find any NAV
        # commands on or after the index. Two scenarios:
        # 1) AUTO mission triggered: The the set_command will fail and it will cause an RTL event
        #       (Harder to test, need vehicle to actually reach the waypoint)
        # 2) GCS/MAVLink: It will return MAV_RESULT_FAILED and there's on change to the mission. (Easy to test)
        self.renumber_mission_items(mission)
        self.check_mission_upload_download(mission)
        self.progress("Checking correct tag behaviour")
        self.change_mode('AUTO')
        self.arm_vehicle()
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_JUMP_TAG,
            p1=17,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED
        )
        self.disarm_vehicle()

    @testcase(bucket="PlaneTests1b")
    def MissionJumpTags(self):
        '''test MAV_CMD_JUMP_TAG'''
        self.wait_ready_to_arm()
        self.MissionJumpTags_missing_jump_target()
        self.MissionJumpTags_do_jump_to_bad_tag()
        self.MissionJumpTags_jump_tag_at_end_of_mission()
