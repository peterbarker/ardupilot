'''
Copter autotest: mission_NAV_LOITER_TURNS_speed

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class mission_NAV_LOITER_TURNS_speed(TestGroup):
    @testcase(bucket="CopterTests2d")
    def mission_NAV_LOITER_TURNS_speed(self):
        '''LOITER_TURNS orbits at WP_SPD, not at CIRCLE_RATE * radius'''
        radius = 20
        # CIRCLE_RATE is deliberately far too low to produce this speed: the orbit must ignore
        # it and fly at WP_SPD, bounded only by the corner acceleration.  Deriving the speed
        # from the rate, as the orbit used to, would give radians(2) * 20 = 0.7 m/s.
        # WP_SPD is set explicitly so the check below does not track its default.
        self.set_parameters({
            'AUTO_OPTIONS': 3,
            'CIRCLE_RATE': 2,
            'WP_SPD': 5,
        })
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                p1=1,
                p3=radius,
                z=30,  # circle is 10m higher than takeoff
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # the leg out to the circle edge also flies at WP_SPD, and ends stopped on the edge, so
        # only sample the speed once the orbit itself has been announced
        self.wait_statustext("Mission: circling", timeout=120)
        tstart = self.get_sim_time()

        # hold the orbit speed while still on the LOITER_TURNS command.  The minimum_duration and
        # the seq assertion together stop a following leg from satisfying this: RTL also flies at
        # WP_SPD, so without them an orbit that is skipped entirely still passes.
        self.wait_groundspeed(4.0, 6.0, timeout=30, minimum_duration=5)
        self.assert_current_waypoint(2)

        # the command must also last about one circumference (2*pi*20 / 5 = 25s).  A skipped
        # orbit reaches RTL in about a tick; the old rate-derived 0.7 m/s would take ~180s.
        self.wait_current_waypoint(3, timeout=120)
        orbit_time_s = self.get_sim_time_cached() - tstart
        expected_s = 2 * math.pi * radius / 5.0
        if orbit_time_s < expected_s * 0.6 or orbit_time_s > expected_s * 2.0:
            raise NotAchievedException(
                "orbit took %.1fs, expected about %.1fs" % (orbit_time_s, expected_s))

        self.wait_disarmed()
