'''
Plane autotest: PlaneDAAFenceBreachEscape

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAFenceBreachEscape(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAFenceBreachEscape(self):
        '''planedaa must not trap the plane inside an exclusion fence after a breach.
        Home is inside a large exclusion circle.  The fence is enabled after takeoff
        so arming is normal.  Once planedaa starts (after STARTUP_DELAY) and the fence
        is breached, fence:get_breaches() is non-zero; the fix causes it to skip fence
        avoidance so the plane can reach the waypoint outside the fence.  Without the
        fix the bendy ruler traps the plane inside and the test times out.'''
        self.install_planedaa_scripts()
        self.context_collect('STATUSTEXT')

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "AVD_ENABLE": 1,
            "FENCE_ENABLE": 0,   # start OFF so arming is not blocked
            "FENCE_TYPE": 4,     # polyfence only (includes MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION)
            "FENCE_ACTION": 0,   # report only — do not RTL on breach
        })

        # exclusion circle centred 200 m north of home, radius 400 m.
        # home is 200 m from the centre — well inside the exclusion.
        home = self.home_position_as_location()
        circle_centre = self.offset_location_ne(home, 200, 0)
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            {"radius": 400, "loc": circle_centre},
        )])

        self.reboot_sitl()

        # This test is about the fix that lets avoidance skip a breached fence, not the
        # trapped failsafe - home is deliberately inside the exclusion the whole time,
        # which the trap would otherwise treat as a sustained compromise and RTL. The
        # script registers this parameter when it loads, so it can only be set post-boot.
        self.set_parameter("DAA_TRAP_ACT", 0)

        # mission: takeoff → waypoint 800 m north (outside the fence) → RTL
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        # Enable the fence once the plane is airborne.  Home is inside the
        # exclusion circle so the breach is immediate — no force-arm needed.
        self.wait_altitude(10, 1000, relative=True, timeout=30)
        self.do_fence_enable()

        # wait for planedaa to announce it is active (after its internal STARTUP_DELAY)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # if the fix is absent, bendy ruler traps the plane inside the fence and
        # the current waypoint never advances past 2; the timeout will fire.
        self.wait_current_waypoint(3, timeout=150)

        self.do_fence_disable()
        self.disarm_vehicle(force=True)
