'''
Plane autotest: PlaneDAAFenceLabelScoped

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAFenceLabelScoped(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAFenceLabelScoped(self):
        '''The AVOIDING distance for a fence must belong to the SAME fence category the
        message labels.  fence_distance() used to return the nearest fence boundary of any
        type, so with more than one fence type present the "Excl. Circle" line could show
        the distance to a nearer inclusion polygon instead (seen in flight in a fence
        cluster).  This loads two fence types at once - an exclusion circle across the path
        (which the plane detours around, so the messages are labelled "Excl. Circle") and a
        large inclusion polygon whose west edge runs ~120 m to the side of the leg (always
        nearer than the circle during the approach, but never the obstacle being routed
        around).  As the plane approaches, the true circle-edge distance is large (>200 m)
        while the polygon edge stays ~120 m.  So a correctly-scoped report produces at least
        one "Excl. Circle dist: >200 m"; the old unscoped behaviour never could (it would
        report the ~120 m polygon under the circle label).'''
        import re
        self.install_planedaa_scripts()
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "AVD_ENABLE": 1,
            "FENCE_ENABLE": 0,
            "FENCE_ACTION": 0,   # report only - a breach fails the test rather than RTL
            "FENCE_TYPE": 4,
        })
        home = self.home_position_as_location()
        fences = [
            # exclusion circle sitting across the WP-A -> WP-B leg (the plane detours it)
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
             {"radius": 60, "loc": self.offset_location_ne(home, 200, 0)}),
            # large inclusion polygon; its west edge runs 120 m west of the leg, so it is the
            # nearest boundary during the approach but is never routed around (leg is parallel,
            # and the detour is eastward, away from it)
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                self.offset_location_ne(home, 700, -120),
                self.offset_location_ne(home, 700, 600),
                self.offset_location_ne(home, -300, 600),
                self.offset_location_ne(home, -300, -120),
            ]),
        ]
        self.upload_fences_from_locations(fences)
        # collect before reboot so the script's start-up announcement is captured
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameter("DAA_MARGIN_FENCE", 50)
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 600, 0, 80),     # 2 north run-in
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -150, 0, 80),    # 3 south, past the circle
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 600, 0, 80),     # 4 north again
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -150, 0, 80),    # 5 south again
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.do_fence_enable()
        # Two passes at the circle, not one.  The AVOIDING message is rate limited to one
        # per 5 s, and the inclusion polygon and the exclusion circle are closely ranked
        # through the approach, so a single pass can easily yield no circle-labelled
        # message at all and the test then fails on an empty list rather than on a wrong
        # distance.  Observed failing in-suite while passing standalone, on unmodified
        # code.  A second pass roughly doubles the samples for ~3 s of extra run time.
        self.wait_current_waypoint(6, timeout=600)   # both passes done -> RTL
        circ_dists = []
        for st in self.context_collection('STATUSTEXT'):
            mm = re.search(r'AVOIDING: Excl. Circle dist: (\d+)m', st.text)
            if mm:
                circ_dists.append(int(mm.group(1)))
        self.progress("Excl. Circle AVOIDING distances: %s" % circ_dists)
        if not circ_dists:
            raise NotAchievedException("no 'Excl. Circle' AVOIDING messages captured")
        if max(circ_dists) <= 200:
            raise NotAchievedException(
                "Excl. Circle distances never exceeded 200 m (max %d) - fence_distance is "
                "not scoped to the labelled fence type (reporting the nearer polygon)"
                % max(circ_dists))
        self.do_fence_disable()
        self.fly_home_land_and_disarm()
        self.do_fence_disable()
        self.disarm_vehicle(force=True)
