'''
Plane autotest: MAV_CMD_DO_SET_MISSION_CURRENT

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_SET_MISSION_CURRENT(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def MAV_CMD_DO_SET_MISSION_CURRENT(self):
        '''test changing the mission item being flown with MAV_CMD_DO_SET_MISSION_CURRENT'''
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),       # 1
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, 50),    # 2
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 800, 50),  # 3
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 800, 50),    # 4
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 50),      # 5
        ])
        self.wait_current_waypoint(2, timeout=60)

        self.start_subtest("invalid sequence numbers")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
            p1=-2,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
            p1=100,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )

        self.start_subtest("change item while flying the mission")
        for run_cmd, seq in (self.run_cmd, 4), (self.run_cmd_int, 3):
            run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT, p1=seq)
            self.wait_current_waypoint(seq, timeout=10)
            self.wait_distance_to_waypoint(seq, 0, 100, timeout=120)

        self.fly_home_land_and_disarm()
