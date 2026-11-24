'''
Plane autotest: PlaneDAAFenceAvoidance

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAFenceAvoidance(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAFenceAvoidance(self):
        '''planedaa must avoid every fence type reported by OAScripting:find_threats().
        Exclusion fences (circle and polygon) sit across the path to a waypoint
        1 km north; the plane must detour around them and still reach the
        waypoint without ever breaching.  Inclusion fences (polyfence circle,
        polyfence polygon and the FENCE_TYPE=2 home circle) surround home with
        the waypoint outside; the plane must approach the boundary but stay
        contained, again without ever breaching.  Each scenario also asserts that the
        obstacle ALERT reaches the GCS, which catches the whole message path being
        suppressed.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "AVD_ENABLE": 1,
            "FENCE_ENABLE": 0,   # enabled in-flight so arming is unimpeded
            "FENCE_ACTION": 0,   # report only — a breach must fail the test, not RTL
        })

        home = self.home_position_as_location()

        # the takeoff climb covers several hundred metres before the mission
        # navigation (and with it DAA avoidance) takes over, so the obstacles
        # sit beyond 1 km and the fence is only enabled once takeoff completes.
        excl_circle = [(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            {"radius": 200, "loc": self.offset_location_ne(home, 1200, 0)},
        )]
        excl_polygon = [(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                self.offset_location_ne(home, 1000, -200),
                self.offset_location_ne(home, 1000, 200),
                self.offset_location_ne(home, 1400, 200),
                self.offset_location_ne(home, 1400, -200),
            ],
        )]
        incl_circle = [(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
            {"radius": 1200, "loc": home},
        )]
        incl_polygon = [(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                self.offset_location_ne(home, 1200, -1200),
                self.offset_location_ne(home, 1200, 1200),
                self.offset_location_ne(home, -1200, 1200),
                self.offset_location_ne(home, -1200, -1200),
            ],
        )]

        # (name, FENCE_TYPE, fence items (None means home circle), expectation)
        scenarios = [
            ("circle exclusion", 4, excl_circle, "detour"),
            ("polygon exclusion", 4, excl_polygon, "detour"),
            ("circle inclusion", 4, incl_circle, "contain"),
            ("polygon inclusion", 4, incl_polygon, "contain"),
            ("home circle", 2, None, "contain"),
        ]

        for (name, fence_type, fence_items, expectation) in scenarios:
            self.start_subtest("planedaa avoids %s fence" % name)
            self.context_push()
            self.context_collect('STATUSTEXT')

            self.set_parameters({
                "FENCE_TYPE": fence_type,
                "FENCE_RADIUS": 1200,  # only used by the home circle fence
            })
            if fence_items is None:
                self.clear_fence()
            else:
                self.upload_fences_from_locations(fence_items)

            self.reboot_sitl()
            self.wait_ready_to_arm()

            # the script registers its parameters when it loads at boot.
            # give the bendy ruler extra clearance: the applet can erode
            # most of the margin while skirting along a fence boundary.
            self.set_parameter("DAA_MARGIN_FENCE", 100)

            # mission: takeoff then a waypoint 2 km north, then RTL.  The
            # exclusion fences make the waypoint reachable only via a detour;
            # the inclusion fences put it out of reach entirely.
            self.start_flying_simple_relhome_mission([
                (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
                (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 80),
                (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
            ])

            # enable the fence once takeoff is complete; during the takeoff
            # climb the plane tracks the runway heading and cannot dodge.
            self.wait_current_waypoint(2, timeout=120)
            self.do_fence_enable()

            # planedaa announces itself once its STARTUP_DELAY has elapsed
            self.wait_text("Plane DAA", check_context=True, timeout=60)

            if expectation == "detour":
                # the plane must dodge the obstacle and still reach the waypoint
                self.wait_current_waypoint(3, timeout=400)
            else:
                # the plane must approach the boundary but never cross it
                self.wait_distance_to_home(700, 100000, timeout=240)
                self.delay_sim_time(60, reason="ensure containment holds")

            if self.statustext_in_collections("fence breached") is not None:
                raise NotAchievedException(
                    "Fence breached during %s scenario" % name)

            # the pilot must actually be told.  alert_obstacle() suppresses the
            # ALERT when the reported range exceeds DAA_DETECT_M, and a fence carries
            # no single location, so this regressed silently once before: the range
            # was measured to an unset Location (lat/lng 0,0) and every fence ALERT
            # was dropped.  Nothing else in the suite reads a DAA message body.
            if self.statustext_in_collections("ALERT:") is None:
                raise NotAchievedException(
                    "no obstacle ALERT sent for %s scenario" % name)

            self.do_fence_disable()
            self.disarm_vehicle(force=True)
            self.context_pop()
