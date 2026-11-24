'''
Rover autotest: MAV_CMD_DO_SET_MISSION_CURRENT_looped_mission

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_SET_MISSION_CURRENT_looped_mission(TestGroup):
    @testcase(bucket="Rover")
    def MAV_CMD_DO_SET_MISSION_CURRENT_looped_mission(self):
        '''Test param1=-1 with reset=1 resets DO_JUMP counter without changing current waypoint'''
        # home(0), WP1(1,30m N), WP2(2,60m N), WP3(3,90m N), DO_JUMP(4,to=2,repeat=1)
        # The jump goes back to WP2 (not WP1), making it clear that
        # param1=-1 resets the counter rather than setting the current waypoint.
        self.start_driving_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 30, 0, 0),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 60, 0, 0),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 90, 0, 0),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_JUMP,
                p1=2,  # jump to item 2 (WP2)
                p2=1,  # repeat once
            ),
        ])

        # Wait for WP3 to be reached the first time; at this point the
        # DO_JUMP has fired and current_waypoint is back at 2 (WP2).
        while True:
            m = self.assert_receive_message('MISSION_ITEM_REACHED', timeout=120)
            if m.seq == 3:
                break

        # Reset jump counters without changing the current waypoint.
        # Current is 2 because the DO_JUMP just looped us back to WP2,
        # not because we are setting it via this command.
        self.wait_current_waypoint(2, timeout=10)
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
            p1=-1,
            p2=1,
            timeout=1,
        )
        self.assert_current_waypoint(2)

        # With the counter reset, DO_JUMP fires again when WP3 is next
        # reached.  Without the reset the mission would complete after
        # WP3 without returning to WP2.  Wait for WP2 then WP3 to
        # confirm the extra loop occurred.
        while True:
            m = self.assert_receive_message('MISSION_ITEM_REACHED', timeout=120)
            if m.seq == 2:
                break
        while True:
            m = self.assert_receive_message('MISSION_ITEM_REACHED', timeout=120)
            if m.seq == 3:
                break

        self.disarm_vehicle(force=True)
