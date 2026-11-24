'''
Copter autotest: MissionUnstartableNavCommandLoop

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MissionUnstartableNavCommandLoop(TestGroup):
    @testcase(bucket="CopterTests2a")
    def MissionUnstartableNavCommandLoop(self):
        '''Test a mission ends when its jump loop holds only a nav command the vehicle cannot start'''

        # the vehicle skips a nav command it cannot start and carries on to
        # the next command.  The mission is meant to fly back and forth
        # between two waypoints but the jump mistakenly targets a plane-only
        # command that Copter cannot start, so every pass of the search skips
        # it and follows the jump back to it.  The search must give up and
        # end the mission rather than start again on every update
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),                                     # 1
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),                                   # 2
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 40, 0, 20),                                   # 3
            (mavutil.mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT, 0, 0, 30),                     # 4 plane only
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_JUMP, p1=4, p2=-1),          # 5 jump to 4 forever
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),                             # 6
        ])

        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_waypoint(3, 3)

        self.progress("Waiting for the mission to complete")
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > 60:
                raise NotAchievedException("Mission did not complete")
            m = self.assert_receive_message('MISSION_CURRENT')
            if m.mission_state == mavutil.mavlink.MISSION_STATE_COMPLETE:
                break

        self.do_RTL()
