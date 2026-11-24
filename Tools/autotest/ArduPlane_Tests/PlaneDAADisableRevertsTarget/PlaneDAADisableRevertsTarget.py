'''
Plane autotest: PlaneDAADisableRevertsTarget

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAADisableRevertsTarget(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAADisableRevertsTarget(self):
        '''planedaa steers by hijacking the vehicle's active navigation target in
        place (vehicle:update_target_location), which also works in RTL.  If the
        pilot switches DAA off (RC aux function 308) while it is mid-avoidance, the
        hijacked target must be reverted to the real navigation target - otherwise
        the vehicle keeps flying to a now-stale avoidance waypoint.  This reproduces
        a field incident: RTL was commanded while DAA was avoiding an exclusion
        circle, DAA was then switched off, and the plane flew straight past home to
        the stale avoidance point instead of returning home.

        An exclusion circle sits on the return path between an outbound waypoint and
        home.  The plane flies out past the circle, is put into RTL, re-encounters
        the circle on the way back so DAA starts avoiding, and DAA is then switched
        off.  With the fix the plane reverts to the home target and returns home;
        without it the plane never reaches home.'''
        self.install_planedaa_scripts()
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "AVD_ENABLE": 1,
            "FENCE_ENABLE": 0,   # enabled in-flight so arming is unimpeded
            "FENCE_ACTION": 0,   # report only - a fence-triggered RTL must not confuse the test
            "FENCE_TYPE": 4,     # polygon/circle fences
            "RC7_OPTION": 308,   # DAA on/off aux switch (DAA_ACT_FN); low = on, high = off
            "RTL_RADIUS": 60,    # tight, predictable home loiter for the distance assertions
        })

        home = self.home_position_as_location()
        # exclusion circle on the home<->waypoint line, so it is crossed on the way
        # out and again on the RTL return leg.
        excl = [(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            {"radius": 200, "loc": self.offset_location_ne(home, 1000, 0)},
        )]
        self.upload_fences_from_locations(excl)
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameter("DAA_MARGIN_FENCE", 100)
        # hold the DAA switch low so it stays enabled (a mid/high switch at boot
        # would toggle it off on the first loop).
        self.set_rc(7, 1000)

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1500, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        # enable the fence once takeoff is complete; during the takeoff climb the
        # plane tracks the runway heading and cannot dodge.
        self.wait_current_waypoint(2, timeout=120)
        self.do_fence_enable()
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # fly out beyond the exclusion circle (centre 1000 m, radius 200 m) so the
        # RTL return leg re-encounters it.
        self.wait_distance_to_home(1300, 100000, timeout=300)

        # return home: the direct path back crosses the exclusion circle, so DAA
        # starts avoiding and hijacks the RTL-to-home target.
        self.change_mode("RTL")
        self.wait_text("AVOIDING", check_context=True, timeout=90)

        # switch DAA off mid-avoidance (the field-incident trigger).
        self.set_rc(7, 2000)

        # the fix reverts the hijacked target; confirm the revert fired and that
        # the plane actually returns home rather than flying on to the stale point.
        self.wait_text("avoidance cleared", check_context=True, timeout=30)
        self.wait_distance_to_home(0, 150, timeout=200)

        # and that it stays at home (loitering), not merely clipping past it.
        self.delay_sim_time(30, reason="confirm the plane holds at home, not flies past")
        dist = self.distance_to_home(use_cached_home=True)
        if dist > 250:
            raise NotAchievedException(
                "plane did not hold at home after DAA disable: %.0f m away" % dist)
        self.do_fence_disable()
        self.disarm_vehicle(force=True)
