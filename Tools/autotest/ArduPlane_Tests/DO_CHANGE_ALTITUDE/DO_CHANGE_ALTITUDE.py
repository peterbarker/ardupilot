'''
Plane autotest: DO_CHANGE_ALTITUDE

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class DO_CHANGE_ALTITUDE(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def DO_CHANGE_ALTITUDE(self):
        '''test DO_CHANGE_ALTITUDE mavlink command'''
        self.install_terrain_handlers_context()

        takeoff_alt = 30
        self.takeoff(alt=takeoff_alt, mode='TAKEOFF')
        # cope with overshoot issues:
        self.wait_altitude(takeoff_alt-2, takeoff_alt+2, minimum_duration=8, relative=True, timeout=90)
        self.wait_altitude(takeoff_alt-1, takeoff_alt+1, minimum_duration=10, relative=True, timeout=60)

        self.start_subtest("Home-relative altitude")
        target_rel_alt = 40
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
            p1=target_rel_alt,
            p2=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        self.wait_altitude(
            target_rel_alt-1,
            target_rel_alt+1,
            minimum_duration=10,
            relative=True,
            timeout=60,
        )

        self.start_subtest("Absolute altitude")
        current_abs_alt = self.get_altitude()
        target_abs_alt = current_abs_alt + 30
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
            p1=target_abs_alt,
            p2=mavutil.mavlink.MAV_FRAME_GLOBAL,
        )
        self.wait_altitude(
            target_abs_alt-1,
            target_abs_alt+1,
            minimum_duration=10,
            timeout=60,
        )

        self.start_subtest("Terrain altitude")
        current_relative_alt = self.get_altitude(relative=True)
        target_terr_alt = current_relative_alt + 10
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
            p1=target_terr_alt,
            p2=mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
        )
        self.wait_altitude(
            target_terr_alt-1,
            target_terr_alt+1,
            minimum_duration=10,
            relative=True,
            altitude_source="TERRAIN_REPORT.current_height",
            timeout=120,
        )

        self.start_subtest("Change alt in loiter")
        self.change_mode('LOITER')
        current_relative_alt = self.get_altitude(relative=True)
        self.wait_altitude(
            current_relative_alt-1,
            current_relative_alt+1,
            minimum_duration=10,
            relative=True,
            timeout=60,
        )
        target_loiter_alt = current_relative_alt+5
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
            p1=target_loiter_alt,
            p2=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        self.wait_altitude(
            target_loiter_alt-1,
            target_loiter_alt+1,
            minimum_duration=10,
            relative=True,
            timeout=60,
        )

        self.start_subtest("Change alt in circle")
        self.change_mode('CIRCLE')
        current_relative_alt = self.get_altitude(relative=True)
        self.wait_altitude(
            current_relative_alt-1,
            current_relative_alt+1,
            minimum_duration=10,
            relative=True,
            timeout=60,
        )
        target_circle_alt = current_relative_alt-5
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
            p1=target_circle_alt,
            p2=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        self.wait_altitude(
            target_circle_alt-1,
            target_circle_alt+1,
            minimum_duration=10,
            relative=True,
            timeout=60,
        )

        self.start_subtest("Immediately respond to DO_CHANGE_ALTITUDE in a mission")
        current_relative_alt = self.get_altitude(relative=True)
        mission_alt = current_relative_alt - 10
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -2000, 0, mission_alt)
        ])
        self.change_mode('AUTO')
        self.wait_altitude(
            mission_alt-1,
            mission_alt+1,
            minimum_duration=10,
            relative=True,
            timeout=240,
        )
        self.fly_home_land_and_disarm()
