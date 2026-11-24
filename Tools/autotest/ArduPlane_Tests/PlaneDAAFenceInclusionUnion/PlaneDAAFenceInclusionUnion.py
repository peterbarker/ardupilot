'''
Plane autotest: PlaneDAAFenceInclusionUnion

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAFenceInclusionUnion(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAFenceInclusionUnion(self):
        '''With FENCE_OPTIONS INCLUSION_UNION (bit 1) set, being inside any ONE inclusion
        area is legal, so DAA must not treat the areas it is legitimately outside as
        blocking fences.

        The scripted distance helpers took the minimum signed clearance across every
        inclusion area, which is right for the default intersection semantics but exactly
        wrong for a union: with two disjoint areas the one the vehicle is not required to
        be inside reports a large negative clearance and the bendy ruler tries to avoid
        it.  Nothing downstream corrects that, because AC_Fence evaluates the union
        properly and so reports no breach - which means the applet's breach-escape (which
        drops all fence avoidance while breached) never engages either.

        Two disjoint inclusion circles are loaded, the vehicle flies well inside the
        first, and no avoidance may occur at all.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "AVD_ENABLE": 1,
            "FENCE_ENABLE": 0,   # enabled in flight so arming is unimpeded
            "FENCE_ACTION": 0,   # report only - a breach must not RTL
            "FENCE_TYPE": 4,     # polygon/circle polyfences (a circle is a polygon here)
            "FENCE_OPTIONS": 2,  # INCLUSION_UNION: inside ANY inclusion area is legal
        })

        home = self.home_position_as_location()
        # the flown area: large enough that the step-1 lookahead never reaches its
        # boundary, so this circle itself is never a legitimate obstacle
        near_circle = (mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                       {"radius": 3000, "loc": home})
        # a second, entirely disjoint inclusion area 8 km north (3000 + 500 << 8000).
        # Under union semantics the vehicle is perfectly legal ignoring this one.
        far_circle = (mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                      {"radius": 500, "loc": self.offset_location_ne(home, 8000, 0)})
        self.upload_fences_from_locations([near_circle, far_circle])

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameter("DAA_MARGIN_FENCE", 100)

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 500, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.do_fence_enable()
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # AC_Fence must agree the union is satisfied; if it reported a breach the applet
        # would drop fence avoidance wholesale and this test would pass for free.
        if self.statustext_in_collections("fence breached") is not None:
            raise NotAchievedException(
                "AC_Fence reported a breach while inside one of two union inclusion areas")

        # fly the legs; nothing here is a real obstacle, so any avoidance is the bug
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 120:
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if m is not None and "AVOIDING" in m.text:
                raise NotAchievedException(
                    "avoided a fence while legally inside a union inclusion area: %s"
                    % m.text.strip())
            if self.current_waypoint() >= 4:
                break

        if self.statustext_in_collections("fence breached") is not None:
            raise NotAchievedException("fence breached during the union flight")
        self.do_fence_disable()
        self.disarm_vehicle(force=True)
