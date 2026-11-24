'''
Plane autotest: PlaneDAAFenceDriftReversal

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAFenceDriftReversal(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAFenceDriftReversal(self):
        '''Regression guard for a tight-margin repeated fence encounter.

        resist_fence_bearing_change() lets a SAME-direction refinement through every
        cycle unresisted (deliberately - that is what fixed an earlier 16-23s
        straight-past-the-fence regression, see the function's own comment). Nothing
        bounds how far that creep can walk clearance down before a reversal becomes
        unavoidable: confirmed live 2026-09-04 (log 00000175.BIN) on a repeating
        racetrack around a single exclusion circle - several clean laps, then the
        committed bearing eroded from a few metres of clearance to a breach in under a
        second as it finally ran out of room on one side and the reversal needed to
        escape could not complete in time.

        This flies a racetrack (DO_JUMP) around one exclusion circle at
        DAA_MARGIN_FENCE=0 (the real turn-radius fallback, a thin margin - little room
        to erode before the physical boundary itself is breached), close enough to
        force a genuine detour every lap without making the geometry an
        unreachable-waypoint/boxed-in scenario like PlaneDAAHungTrapFires.  It does NOT
        reproduce the field breach on demand: several tuned geometries (a direct
        crossing, a tangent flyby, added SIM_WIND_TURB to break the lap-to-lap
        determinism a clean synthetic racetrack otherwise has) all settled to a stable,
        safe minimum clearance every lap rather than eroding, on both the fixed and
        unfixed code - the field failure likely depends on real flight variability
        (wind, VTOL transition dynamics, or state built up over genuinely non-identical
        laps) that this cannot cheaply recreate.  Kept as a real, if not
        bug-discriminating, safety margin check; retune the geometry (or replace this
        with a direct assertion on DAAD.HdgB/DstB from a captured dataflash log) if a
        reproduction is found later. FENCE_ACTION=0 (report only) so a breach fails the
        test outright rather than being masked by the core's own RTL recovery.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "AVD_ENABLE": 1,
            "FENCE_ENABLE": 0,      # enabled in-flight so arming is unimpeded
            "FENCE_TYPE": 4,        # polyfence (circle exclusion)
            "FENCE_ACTION": 0,      # report only - a breach must fail the test, not RTL
            "ROLL_LIMIT_DEG": 65,
            "AIRSPEED_CRUISE": 25,
            # light turbulence, matching the field flight (log 00000175.BIN) - included
            # for parity with the params that actually reproduced the breach, though on
            # its own it was not enough to break this test's lap-to-lap determinism (see
            # the docstring above):
            "SIM_WIND_SPD": 0,
            "SIM_WIND_TURB": 2,
            # ArduPlane's own default - "rate limit disabled" - the actual case the
            # roll-rate fallback must cover:
            "RLL2SRV_RMAX": 0,
            "RLL2SRV_TCONST": 0.5,
        })

        home = self.home_position_as_location()
        # exclusion circle centred on the WP2->WP3 leg (both waypoints comfortably
        # outside it - this is a detour, not an unreachable-waypoint/boxed-in scenario
        # like PlaneDAAHungTrapFires): the leg's direct line runs straight through the
        # circle, forcing a genuine detour every lap, thin enough at DAA_MARGIN_FENCE=0
        # (the real turn-radius fallback, below) to hug the boundary closely without
        # making the encounter geometrically impossible.
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            {"radius": 150, "loc": self.offset_location_ne(home, 520, 250)},
        )])

        # collect before reboot so the script's start-up announcement is captured
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        # DAA_ params only register once the script has loaded post-boot.
        self.set_parameters({
            # real turn-radius fallback - the thin margin that actually reproduced this
            # in the field:
            "DAA_MARGIN_FENCE": 0,
            "DAA_PLAN_M": 200,
            "DAA_LKAHD_M": 500,
        })

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 700, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 700, 500, 80),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 300, 500, 80),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 300, 0, 80),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_JUMP,
                p1=2,   # jump back to the first racetrack leg
                p2=20,  # plenty of laps for the timeout below
            ),
        ])

        # enabled once the climb is done; during takeoff the plane tracks the runway
        # heading and cannot dodge.
        self.wait_current_waypoint(2, timeout=120)
        self.do_fence_enable()
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        self.delay_sim_time(120, reason="run the racetrack past the circle several laps")

        if self.statustext_in_collections("fence breached") is not None:
            raise NotAchievedException("Fence breached on the racetrack")

        self.do_fence_disable()
        self.disarm_vehicle(force=True)
