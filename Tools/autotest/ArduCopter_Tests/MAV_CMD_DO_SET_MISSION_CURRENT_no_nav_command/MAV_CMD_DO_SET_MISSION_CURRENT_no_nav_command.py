'''
Copter autotest: MAV_CMD_DO_SET_MISSION_CURRENT_no_nav_command

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_SET_MISSION_CURRENT_no_nav_command(TestGroup):
    @testcase(bucket="CopterTests2a")
    def MAV_CMD_DO_SET_MISSION_CURRENT_no_nav_command(self):
        '''Test setting the current command in a mission whose loop holds no nav command'''

        # setting the current command searches forward for a nav command to
        # load, following jumps as it goes.  A forever jump over nothing but
        # do commands never runs out of repeats and never reaches a nav
        # command, so the search has nothing to terminate on and locks up the
        # main loop, taking the vehicle with it.
        # The mission is meant to fly back and forth between two waypoints
        # but the jump mistakenly targets the DO_SET_ROI_NONE instead of the
        # first waypoint, and the RTL after it can never be reached
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),                                    # 1
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),                                  # 2
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 40, 0, 20),                                  # 3
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE),              # 4
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_JUMP, p1=4, p2=-1),         # 5 jump to 4 forever
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),                            # 6
        ])

        # the vehicle must answer.  A missing ack here means it never came back
        # from the search, not that it declined the command
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
            p1=4,
            timeout=10,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )

        # and must still be talking to us afterwards
        self.wait_heartbeat()
        self.wait_heartbeat()
