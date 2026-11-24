'''
Copter autotest: WPArcs2

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class WPArcs2(TestGroup):
    @testcase(bucket="CopterTests1a")
    def WPArcs2(self):
        '''more tests for waypoint arcs'''
        self.set_parameters({
            "AUTO_OPTIONS": 3,
        })

        def assert_no_movement(mav, m):
            m_type = m.get_type()
            if m_type != 'VFR_HUD':
                return
            if abs(m.climb) > 0.2 or abs(m.groundspeed) > 0.2:
                raise NotAchievedException(f"Moved when I shouldn't {m=}")

        def assert_no_lateral_movement(mav, m):
            m_type = m.get_type()
            if m_type != 'VFR_HUD':
                return
            print(f"{m.groundspeed=}")
            if abs(m.groundspeed) > 0.2:
                raise NotAchievedException(f"Moved when I shouldn't {m=}")

        self.start_subtest("Shouldn't move if no takeoff")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        self.install_message_hook_context(assert_no_movement)
        self.fly_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 0, 0, 20, {"p1": 90}),
        ])

        self.context_pop()

        self.start_subtest("Shouldn't move if no takeoff")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        self.install_message_hook_context(assert_no_movement)
        self.fly_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 20, 20, 20, {"p1": 90}),
        ])
        self.context_pop()

        self.context_push()
        self.start_subtest("Shouldn't move if no takeoff, even if we repeat ourselves")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.install_message_hook_context(assert_no_movement)
        self.fly_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 0, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 0, 0, 20, {"p1": 90}),
        ])
        self.context_pop()

        self.start_subtest("Shouldn't move if no start waypoint")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        self.install_message_hook_context(assert_no_lateral_movement)
        self.fly_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 0, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.context_pop()

        self.start_subtest("Shouldn't move if endpoint is startpoint")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        self.install_message_hook_context(assert_no_lateral_movement)
        self.fly_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 0, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.context_pop()

        self.progress("Creating a log per-flight")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.set_parameter("LOG_FILE_DSRMROT", 1)
        self.arm_vehicle()
        self.disarm_vehicle()

        # this will move but won't arc (it should...):
        self.start_subtest("Should move if no start waypoint")
        self.context_push()
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 30, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_groundspeed(1, 10)
        self.wait_distance_to_home(10, 20)
        self.wait_disarmed()
        self.context_pop()

        self.start_subtest("Degenerate arc - should be a straight-line segment")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 30, 0, 20, {"p1": 0}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_groundspeed(1, 10)
        self.wait_distance_to_home(10, 20)
        self.wait_disarmed()

        self.start_subtest("Degenerate arc - no length")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 10, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 10, 0, 20, {"p1": 180}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_groundspeed(1, 10)
        self.wait_distance_to_home(5, 20)
        self.wait_disarmed()

        self.start_subtest("Should arc if we start with a waypoint")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        pos, radius = self.circle_from_arc((0, 0), (90, 0), 90)
        loc = self.offset_location_ne(self.home_position_as_location(), pos[0], pos[1])
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_altitude(19, 21, relative=True)
        self.wait_groundspeed(1, 10)
        self.wait_circling_point_with_radius(
            loc, radius,
            epsilon=1,
            min_circle_time=7,
            timeout=30,
            track_angle=False,
        )
        self.wait_distance_to_home(80, 100)
        self.wait_disarmed()

        self.start_subtest("arc - should end up with a mirrored D")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        pos, radius = self.circle_from_arc((0, 0), (90, 0), -90)
        loc = self.offset_location_ne(self.home_position_as_location(), pos[0], pos[1])
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90, 0, 20, {"p1": 270}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_altitude(19, 21, relative=True)
        self.wait_groundspeed(1, 10)
        self.wait_circling_point_with_radius(
            loc, radius,
            epsilon=1,
            min_circle_time=10,
            timeout=30,
            track_angle=False,
        )
        self.wait_disarmed(timeout=600)
        self.context_pop()

        self.start_subtest("arc - should end up with a D")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        pos, radius = self.circle_from_arc((0, 0), (90, 0), 90)
        loc = self.offset_location_ne(self.home_position_as_location(), pos[0], pos[1])
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90, 0, 20, {"p1": -270}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_altitude(19, 21, relative=True)
        self.wait_groundspeed(1, 10)
        self.wait_circling_point_with_radius(
            loc, radius,
            epsilon=1,
            min_circle_time=10,
            timeout=30,
            track_angle=False,
        )
        self.wait_disarmed(timeout=600)
        self.context_pop()

        self.start_subtest("arc - should end up with helix up")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        pos = (45, 0)
        radius = 45
        loc = self.offset_location_ne(self.home_position_as_location(), pos[0], pos[1])
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90, 0, 100, {"p1": -2700}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_altitude(19, 21, relative=True)
        self.wait_groundspeed(1, 10)
        self.wait_circling_point_with_radius(
            loc, radius,
            epsilon=1,
            min_circle_time=120,
            timeout=240,
            track_angle=False,
        )
        self.wait_altitude(99, 101, relative=True, timeout=240)
        self.wait_disarmed(timeout=600)
        self.context_pop()

        self.start_subtest("arc - should end up with helix down")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 100),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 100),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 30, 0, 20, {"p1": 2700}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_altitude(90, 100, relative=True, timeout=120)
        self.wait_groundspeed(1, 10)
        self.wait_distance_to_home(10, 20)
        self.wait_disarmed(timeout=600)
        self.context_pop()

        self.start_subtest("arc - chained arcs")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 30, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 60, 0, 20, {"p1": 90}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 60, 0, 20, {"p1": -90}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 60, 0, 20, {"p1": -90}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_altitude(15, 25, relative=True, timeout=120)
        self.wait_groundspeed(1, 10)
        self.wait_distance_to_home(10, 20)
        self.wait_disarmed(timeout=600)
        self.context_pop()

        self.start_subtest("arc - chained arcs - up/down")
        self.change_mode('STABILIZE')  # needed to ensure Copter mission state machine works
        self.context_push()
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 30, 0, 20, {"p1": 180}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 60, 0, 40, {"p1": -180}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 90, 0, 40, {"p1": 180}),
            (mavutil.mavlink.MAV_CMD_NAV_ARC_WAYPOINT, 120, 0, 20, {"p1": -180}),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_altitude(15, 25, relative=True, timeout=120)
        self.wait_groundspeed(1, 10)
        self.wait_distance_to_home(10, 20)
        self.wait_disarmed(timeout=600)
        self.context_pop()
