'''
Copter autotest: JumpNextNavCmd

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class JumpNextNavCmd(TestGroup):
    def fly_jump_loop_mission(self, name, items, loop_end_wp, loop_centre):
        '''fly items and check the vehicle keeps to the loop the jump creates

        items must jump back forever over a leg centred on loop_centre.
        loop_end_wp is the sequence number of the last waypoint of that leg,
        reached just before the jump is taken.
        '''
        self.progress("Jump case: %s" % name)
        self.start_flying_simple_relhome_mission(items)

        # reach the end of the looped leg for the first time
        self.wait_current_waypoint(loop_end_wp, timeout=120)

        # the jump goes back to the start of the leg, so the vehicle should
        # orbit it.  Position is checked rather than the reported current
        # waypoint, which advances correctly even when the lookahead does not
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 45:
            dist = self.get_distance(loop_centre, self.get_location())
            if dist > 100:
                raise NotAchievedException(
                    "%s: vehicle left the loop after the jump, %.0fm from its centre" % (name, dist))
            self.delay_sim_time(1, "checking the vehicle stays on the loop")

        self.progress("%s: stayed on the loop" % name)
        self.do_RTL()

    @testcase(bucket="CopterTests2b")
    def JumpNextNavCmd(self):
        '''Verify Copter flies the loop that a DO_JUMP or DO_JUMP_TAG creates'''

        # when flying to a waypoint Copter also looks at the nav command that
        # follows it, to shape the path through the corner.  That look-ahead
        # must find the same command the mission goes on to fly.  Each case
        # below jumps forever back over the wpA <-> wpB leg, so the vehicle
        # must stay near that leg.  The waypoints 300m south of it are only
        # reached if the jump is ignored.  The cases are DO_JUMP and
        # DO_JUMP_TAG, each landing on a nav command or a do command
        alt = 20
        loop_centre = self.offset_location_ne(self.home_position_as_location(), 7, 22)

        takeoff_and_approach = [
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 19, -57, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 35, -17, alt),
        ]
        wpA = (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 23, 14, alt)
        wpB = (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -8, 30, alt)
        beyond_the_jump = [
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -300, 30, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -350, 30, alt),
        ]
        # a do command that Copter's AUTO handles and that needs no hardware
        roi_none = mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE

        # the target is a do command, so the scan has to carry on past it to
        # reach wpA.  This is the general form of the bug, tags aside
        self.fly_jump_loop_mission(
            "jump to do command",
            takeoff_and_approach + [
                self.create_MISSION_ITEM_INT(roi_none),                                      # 4 jump target
                wpA,                                                                         # 5
                wpB,                                                                         # 6
                self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_JUMP, p1=4, p2=-1),  # 7
            ] + beyond_the_jump,
            loop_end_wp=6,
            loop_centre=loop_centre)

        # the target is a nav command, which the scan returns immediately.
        # This shape always worked, and is here to keep it that way
        self.fly_jump_loop_mission(
            "jump to nav command",
            takeoff_and_approach + [
                wpA,                                                                         # 4 jump target
                wpB,                                                                         # 5
                self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_JUMP, p1=4, p2=-1),  # 6
            ] + beyond_the_jump,
            loop_end_wp=5,
            loop_centre=loop_centre)

        # a tag jump always lands on the tag marker, which is never a nav
        # command, so this shape was broken for every mission that used tags
        self.fly_jump_loop_mission(
            "jump tag to nav command",
            takeoff_and_approach + [
                self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_JUMP_TAG, p1=1),            # 4 tag 1
                wpA,                                                                             # 5
                wpB,                                                                             # 6
                self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_JUMP_TAG, p1=1, p2=-1),  # 7
            ] + beyond_the_jump,
            loop_end_wp=6,
            loop_centre=loop_centre)

        # tag marker then a do command: the scan has to step over two non-nav
        # commands before it reaches wpA
        self.fly_jump_loop_mission(
            "jump tag to do command",
            takeoff_and_approach + [
                self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_JUMP_TAG, p1=2),            # 4 tag 2
                self.create_MISSION_ITEM_INT(roi_none),                                          # 5
                wpA,                                                                             # 6
                wpB,                                                                             # 7
                self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_JUMP_TAG, p1=2, p2=-1),  # 8
            ] + beyond_the_jump,
            loop_end_wp=7,
            loop_centre=loop_centre)
