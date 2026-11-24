'''
Plane autotest: PlaneDAASecondLegLookahead

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAASecondLegLookahead(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAASecondLegLookahead(self):
        '''planedaa's bendy-ruler second step must look beyond step 1 and be able to
        reject a heading that leads into a dead end.

        Step 1 covers 0 -> DAA_LKAHD_M ahead of the aircraft; step 2 continues from the
        end of step 1 for a further 2 x DAA_LKAHD_M, probing straight and +/-45 degrees.
        As upstream AP_OABendyRuler does, a heading is only rejected when ALL THREE of
        those continuations are blocked - a lone obstacle in open country always leaves
        a dogleg, so it is handled by step 1 instead.  This test therefore uses a wall
        wide enough that the +/-45 probes land on it too.

        The second step regressed twice over and each fault alone makes it inert: the
        probe collapsed onto a 1 m stub at the far end of the leg (it queried
        loc_test2 -> target_loc, the same point when delta == 0), and its obstacle was
        dropped at the call site, so DAA.detect() - which ends its sweep on a nil
        obstacle - read the blocked heading as clear.  Nothing else in the suite
        notices: the whole PlaneDAA suite passes with the second step doing nothing.

        The visible symptom is the range at which the plane turns away from the wall.'''
        self.install_planedaa_scripts()

        # This test's whole geometry (wall placement, dogleg ranges) is built around a
        # 1000 m probe distance - set it explicitly rather than riding whatever
        # DAA_LKAHD_M's shipping default happens to be, since that default is tuned for
        # flight behaviour, not for this test's fixed wall.
        lookahead_m = 1000
        step2_leg_m = 2 * lookahead_m
        fence_margin_m = 100
        wall_north_m = 4200

        # the +/-45 probes are the last to be blocked: they start at the end of step 1
        # and reach only lookahead + step2_leg*cos(45) northwards
        step2_range_m = (step2_leg_m * math.cos(math.radians(45))
                         + lookahead_m + fence_margin_m)
        # step 1 alone cannot see the wall until it is this close
        step1_only_range_m = lookahead_m + fence_margin_m
        # assert comfortably between the two so neither bound is a near miss
        min_expected_range_m = 0.5 * (step1_only_range_m + step2_range_m)

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "AVD_ENABLE": 1,
            "FENCE_ENABLE": 0,   # enabled in flight so arming is unimpeded
            "FENCE_ACTION": 0,   # report only - a breach must fail the test, not RTL
            "FENCE_TYPE": 4,     # polygon/circle fences only, no home circle
        })

        home = self.home_position_as_location()
        # a wall across the path, wide enough that the +/-45 degree second-leg probes
        # (which reach about 1414 m either side) land on it rather than going round
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                self.offset_location_ne(home, wall_north_m, -2500),
                self.offset_location_ne(home, wall_north_m, 2500),
                self.offset_location_ne(home, wall_north_m + 200, 2500),
                self.offset_location_ne(home, wall_north_m + 200, -2500),
            ],
        )])

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        # the script registers its parameters when it loads at boot
        self.set_parameters({
            "DAA_MARGIN_FENCE": fence_margin_m,
            "DAA_LKAHD_M": lookahead_m,
        })

        # the waypoint is beyond the wall, so the plane keeps pressing north into it
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 5000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        # enable the fence once takeoff is complete; during the takeoff climb the
        # plane tracks the runway heading and cannot dodge
        self.wait_current_waypoint(2, timeout=120)
        self.do_fence_enable()
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        def range_to_wall():
            here = self.get_location()
            return wall_north_m - self.get_distance(home, here)

        # if the takeoff already carried us inside step-1 range the test cannot tell
        # the two behaviours apart, so fail loudly rather than passing for free
        if range_to_wall() <= min_expected_range_m:
            raise NotAchievedException(
                "takeoff ended only %.0f m from the wall, too close to discriminate "
                "(need more than %.0f m)" % (range_to_wall(), min_expected_range_m))

        tstart = self.get_sim_time()
        range_at_first_avoid_m = None
        while self.get_sim_time() - tstart < 400:
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if m is None:
                continue
            if "AVOIDING" in m.text:
                range_at_first_avoid_m = range_to_wall()
                break
        if range_at_first_avoid_m is None:
            raise NotAchievedException("planedaa never avoided the exclusion wall")

        self.progress(
            "first AVOIDING %.0f m from the wall (step 1 alone: about %.0f m, "
            "with a working step 2: about %.0f m)" %
            (range_at_first_avoid_m, step1_only_range_m, step2_range_m))
        if range_at_first_avoid_m < min_expected_range_m:
            raise NotAchievedException(
                "avoidance engaged only %.0f m from the wall: the bendy-ruler second "
                "leg is not rejecting the dead end (expected more than %.0f m)" %
                (range_at_first_avoid_m, min_expected_range_m))

        # having turned away it must not then fly into the wall
        self.delay_sim_time(60, reason="confirm the turn away holds")
        if self.statustext_in_collections("fence breached") is not None:
            raise NotAchievedException("fence breached after avoiding the wall")
        self.do_fence_disable()
        self.disarm_vehicle(force=True)
