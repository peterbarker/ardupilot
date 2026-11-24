'''
Plane autotest: PlaneDAABreachScopedToFenceType

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAABreachScopedToFenceType(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAABreachScopedToFenceType(self):
        '''A breached fence must not mask every other obstacle.

        find_threats() returns the single closest obstacle, and a breached fence reports a
        large NEGATIVE clearance, so while any fence was breached it won that comparison
        every cycle and nothing else - another fence, or traffic - could be returned at
        all.  planedaa then had to drop it (inside an exclusion, or outside an inclusion,
        every route reads as blocked and the bendy ruler would trap the plane trying to get
        back), which left the applet completely blind for as long as the breach lasted.
        The fix skips the breached fence CATEGORIES inside find_threats() so the next real
        obstacle surfaces instead.

        Geometry: a 300 m home inclusion circle (AC_FENCE_TYPE_CIRCLE) and an exclusion
        circle sitting on the leg 1400 m north (AC_FENCE_TYPE_POLYGON - every polyfence
        item breaches under that bit).  The fence is enabled only once the plane is already
        well outside the home circle, so the CIRCLE bit is set for the whole run and never
        clears.  Standing the home circle down is correct - it is the breached one - but
        the exclusion circle is a different category and must still be avoided.

        Asserted physically as well as by message: the plane must never come within the
        exclusion radius.  Before the fix it flies straight through the centre.'''
        self.install_planedaa_scripts()

        home_radius_m = 300
        excl_north_m = 1400
        excl_radius_m = 250

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "AVD_ENABLE": 1,
            "FENCE_ENABLE": 0,      # enabled in flight, once we are outside the home circle
            "FENCE_TYPE": 6,        # 2 home circle | 4 polyfence
            "FENCE_ACTION": 0,      # report only - the home-circle breach is the point
            "FENCE_RADIUS": home_radius_m,
        })

        home = self.home_position_as_location()
        excl_centre = self.offset_location_ne(home, excl_north_m, 0)
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            {"radius": excl_radius_m, "loc": excl_centre},
        )])

        # collect before reboot so the script's start-up announcement is captured
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameters({
            "DAA_MARGIN_FENCE": 100,
            # This test is about find_threats()'s breach-scoping, not the trapped
            # failsafe - the home-circle breach is deliberately sustained for the whole
            # run, which the trap would otherwise treat as a compromise and RTL for.
            "DAA_TRAP_ACT": 0,
        })

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2600, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        # only enable the fence once the home circle is already behind us, so DAA is
        # never asked to keep us inside it and the breach persists for the whole run.
        self.wait_distance_to_home(home_radius_m + 200, 100000, timeout=180)
        self.do_fence_enable()
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        if self.get_distance(home, self.get_location()) < home_radius_m:
            raise NotAchievedException("inside the home circle: it is not breached")

        # record the closest approach to the exclusion circle passively, off the position
        # stream, rather than polling for position while we wait
        closest = [1e9]

        def watch_closest(mav, m):
            if m.get_type() != 'GLOBAL_POSITION_INT':
                return
            here = Location.latlon_only(m.lat * 1e-7, m.lon * 1e-7)
            closest[0] = min(closest[0], self.get_distance(excl_centre, here))

        self.install_message_hook_context(watch_closest)

        # fly the leg out past the exclusion circle
        self.wait_distance_to_home(excl_north_m + excl_radius_m + 300, 100000, timeout=400)

        closest_m = closest[0]
        self.progress("closest approach to the exclusion centre: %.0f m (radius %u m)"
                      % (closest_m, excl_radius_m))

        if closest_m < excl_radius_m:
            raise NotAchievedException(
                "flew inside the exclusion circle (closest %.0f m, radius %u m): the home "
                "circle breach stood down avoidance of an unbreached fence"
                % (closest_m, excl_radius_m))

        if self.statustext_in_collections("AVOIDING: Excl. Circle") is None:
            raise NotAchievedException(
                "no 'AVOIDING: Excl. Circle' report: the exclusion fence was never "
                "avoided while the home circle was breached")

        self.do_fence_disable()
        self.disarm_vehicle(force=True)
