'''
Copter autotest: SplineNavDelay

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SplineNavDelay(TestGroup):
    @testcase(bucket="CopterTests1c")
    def SplineNavDelay(self):
        '''Test spline waypoint followed by NAV_DELAY with ignored time fields'''
        # see https://github.com/ArduPilot/ardupilot/issues/34651 ;
        # NAV_DELAY's hour/min/sec of -1 were read as an altitude
        # for the spline's next control point
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT, 30, 20, 25),
            (mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT, 30, 60, 20),
            (mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT, 0, 80, 15),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_DELAY,
                p1=3,
                p2=-1,
                p3=-1,
                p4=-1,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        # the vehicle must not descend much below the lowest spline
        # waypoint while flying the spline legs
        self.wait_current_waypoint(2, timeout=60)
        self.context_push()
        min_alt = 12

        def check_altitude(mav, m):
            if m.get_type() != 'GLOBAL_POSITION_INT':
                return
            alt = m.relative_alt * 0.001
            if alt < min_alt:
                raise NotAchievedException(f"Spline descended to {alt:.1f}m (want >= {min_alt}m)")

        self.install_message_hook_context(check_altitude)
        self.wait_current_waypoint(5, timeout=120)
        self.context_pop()

        self.wait_disarmed(timeout=120)
