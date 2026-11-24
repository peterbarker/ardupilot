'''
Copter autotest: HomeCircleInclusionFence

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class HomeCircleInclusionFence(TestGroup):
    def fence_item_NAV_FENCE_HOME_CIRCLE_INCLUSION(self, *, radius: float):
        return self.mav.mav.mission_item_int_encode(
            1,  # sysid
            1,  # compid
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            5005,  # mavutil.mavlink.MAV_CMD_NAV_FENCE_HOME_CIRCLE_INCLUSION
            0, # current
            0, # autocontinue
            radius, # p1 - radius
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            0.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )

    @testcase(bucket="CopterTests2d")
    def HomeCircleInclusionFence(self, target_system=1, target_component=1):
        """Home Circle fence using MAV_CMD_NAV_FENCE_HOME_CIRCLE_INCLUSION."""

        """
        Create a home circled fence, attempt to drive outside it, detect return to home.
        Then, clear the fence and achieve the target location.

        ./Tools/autotest/autotest.py test.CopterTests2b.HomeCircleInclusionFence  --map --speedup 1
        """
        self.check_fence_upload_download([
            self.fence_item_NAV_FENCE_HOME_CIRCLE_INCLUSION(radius=50),
        ])

        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_ACTION": 1,  # RTL or Hold
            "FENCE_TYPE":   4,  # polygons only
        })

        self.reboot_sitl()

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,     0,    0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  -25,  -25, 25),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -100, -100, 25),
        ])

        self.start_subtest("Nominal case: fence breach triggers RTL to home")
        # Waypoint 2 location inside the fence
        self.wait_current_waypoint(2)
        self.wait_current_waypoint(3)
        self.wait_statustext("Polygon fence breached")
        # With fence enabled, expect RTL to trigger and the vehicle to go home
        self.wait_rtl_complete(timeout=300)
        self.end_subtest("Nominal case: fence breach triggers RTL to home")

        self.start_subtest("Clear fence: vehicle reaches waypoint 3 target location")
        # Clear the fence and re-run the mission
        self.clear_fence()
        self.set_parameter("FENCE_ENABLE", 0)

        # Waypoint 3 location - reachable now that fence is cleared
        self.change_mode('AUTO')
        self.arm_vehicle()
        self.wait_current_waypoint(3)
        self.wait_distance_to_waypoint(3, distance_min=0, distance_max=5)
        self.do_RTL(distance_max=150)

        self.end_subtest("Clear fence: vehicle reaches waypoint 3 target location without hitting fence")

    @testcase(bucket="CopterTests1a")
    def HomeCircleInclusionFence_SetHome(self, target_system=1, target_component=1):
        '''manipulate home position while using HOME_CIRCLE_FENCE'''
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_ACTION": 1,  # RTL or Hold
            "FENCE_TYPE":   4,  # polygons only
        })
        self.check_fence_upload_download([
            self.fence_item_NAV_FENCE_HOME_CIRCLE_INCLUSION(radius=50),
        ])
        self.takeoff(5, mode='LOITER')
        self.clear_fence()
        self.check_fence_upload_download([
            self.fence_item_NAV_FENCE_HOME_CIRCLE_INCLUSION(radius=50),
        ])
        self.wait_mode('LOITER')
        # be careful of AC_FENCE_GIVE_UP_DISTANCE here (100m default)
        self.progress("Move home 60m North")
        pos = self.offset_location_ne(self.get_location(frame=AltFrame.ABOVE_HOME), 60, 0)
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_home(pos)
        self.poll_home_position()
        self.wait_statustext("Polygon fence breached", check_context=True)
        self.wait_mode('RTL')
        self.wait_rtl_complete(check_alt=False)
        self.context_pop()

    @testcase(bucket="CopterTests1c")
    def HomeCircleInclusionFence_MultipleHomeCircle(self, target_system=1, target_component=1):
        '''upload multiple circles; should breach passing the innermost'''
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_ACTION": 1,  # RTL or Hold
            "FENCE_TYPE":   4,  # polygons only
        })
        items = [
            self.fence_item_NAV_FENCE_HOME_CIRCLE_INCLUSION(radius=50),
            self.fence_item_NAV_FENCE_HOME_CIRCLE_INCLUSION(radius=100),
        ]
        self.correct_wp_seq_numbers(items)
        self.check_fence_upload_download(items)
        self.takeoff(5, mode='STABILIZE')
        self.set_rc(2, 1200)
        self.wait_statustext("Polygon fence breached")
        self.set_rc(2, 1500)
        self.wait_distance_to_home(40, 100, timeout=0.01)
        self.wait_mode('RTL')
        self.wait_disarmed()

    @testcase(bucket="CopterTests1c")
    def HomeCircleInclusionFence_Avoidance(self):
        '''Avoidance holds vehicle inside home-centered circle inclusion fence'''
        fence_radius = 20
        fence_margin = 2
        self.check_fence_upload_download([
            self.fence_item_NAV_FENCE_HOME_CIRCLE_INCLUSION(radius=fence_radius),
        ])
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 4,
            "FENCE_ACTION": 0,   # report only; rely on avoidance
            "FENCE_MARGIN": fence_margin,
            "AVOID_ENABLE": 1,   # AC_AVOID_STOP_AT_FENCE
            "AVOID_BEHAVE": 0,   # slide
        })
        self.takeoff(10, mode='LOITER')
        self.set_rc(2, 1300)
        inner_radius = fence_radius - fence_margin
        self.wait_distance_to_home(
            inner_radius - 1,
            fence_radius + 1,
            timeout=60,
            minimum_duration=20,
        )
        self.wait_groundspeed(0, 0.5, timeout=15)
        self.set_rc(2, 1500)
        self.do_RTL()

    @testcase(bucket="CopterTests1a")
    def HomeCircleInclusionFence_Avoidance_SetHome(self):
        '''Avoidance tracks home-circle fence boundary after home is relocated mid-flight'''
        fence_radius = 20
        fence_margin = 2
        self.check_fence_upload_download([
            self.fence_item_NAV_FENCE_HOME_CIRCLE_INCLUSION(radius=fence_radius),
        ])
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 4,
            "FENCE_ACTION": 0,   # report only; rely on avoidance
            "FENCE_MARGIN": fence_margin,
            "AVOID_ENABLE": 1,   # AC_AVOID_STOP_AT_FENCE
            "AVOID_BEHAVE": 0,   # slide
        })
        self.takeoff(10, mode='LOITER')

        self.context_set_speedup(1)

        # Push west; avoidance should hold vehicle near inner boundary of original fence.
        self.set_rc(2, 1300)
        inner_radius = fence_radius - fence_margin
        self.wait_distance_to_home(
            inner_radius - 1,
            fence_radius + 1,
            timeout=60,
            minimum_duration=20,
        )

        # Move home West. The fence boundary follows home, so avoidance must
        # allow the vehicle to advance to the new boundary. A correct
        # implementation queries AP::ahrs().get_home() for the circle centre;
        # a broken one uses the stale EKF-origin offset and keeps the vehicle
        # pinned to the old boundary (8m short of the new inner_radius).
        new_home = self.offset_location_ne(
            self.home_position_as_location(), 0, -10,
        )
        self.set_home(new_home)
        self.poll_home_position()

        # distance_to_home now measures from the new home.  With correct
        # avoidance the vehicle advances West until it reaches inner_radius
        # from the new home and stabilises there.
        self.wait_distance_to_home(
            inner_radius - 1,
            fence_radius + 1,
            timeout=60,
            minimum_duration=20,
        )
        self.wait_groundspeed(0, 0.5, timeout=15)
        # a forced reboot rather than an RTL: the RTL (at speedup 1)
        # proves nothing further, and set_home requires a post-test
        # reboot anyway
        self.reboot_sitl(force=True)
