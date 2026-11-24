'''
Plane autotest: PlaneDAALoiterFailNoState

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAALoiterFailNoState(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAALoiterFailNoState(self):
        '''When the aircraft loiter cannot start, planedaa must not claim to be loitering.

        loiteralt.start() gives up for several reasons - already active, no current_loc,
        neither side clearing a nearby fence (see PlaneDAALoiterFencePicksClearSide for the
        case where only ONE side is blocked - that one must succeed, not refuse), and
        set_vehicle_target_location() itself being refused - and used to return nil on all
        of them, indistinguishable from success.  Callers set current_state =
        STATE.loitering regardless, so the state machine believed it was loitering while
        loiteralt.active was false and do_loitering() ran against a loiter that never began.

        The refusal is provoked the way it happens in the field.  The loiter point is placed
        WP_LOITER_RAD abeam, and both sides are blocked - a corridor too narrow for either
        the left or right loiter circle to clear it - so there is genuinely nowhere safe to
        loiter, not just a bad first guess. Before self.start()'s own fence-clearance check
        existed, Plane's DO_REPOSITION itself ("reject destination if outside the fence",
        GCS_MAVLink_Plane.cpp) was what refused; now self.start() catches it first, but
        either refusal exercises the same invariant, so this waits for either message.

        The corridor is sized so DAA's own fence avoidance stays out of the way: each wall
        is at least 100 m from track while DAA_MARGIN_FENCE is 20 m, so no fence is ever
        avoided. That matters because current_state == STATE.avoiding would skip the loiter
        branch entirely and the test would prove nothing.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,      # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "AVD_WCLR_XY": 200,
            "AVD_WCLR_Z": 50,
            "FENCE_ENABLE": 0,   # enabled in flight so arming is unimpeded
            "FENCE_ACTION": 0,   # report only
            "FENCE_TYPE": 4,     # polygon fences
            "WP_LOITER_RAD": 400,   # loiter point lands 400 m abeam of track
        })

        home = self.home_position_as_location()
        # a corridor the aircraft flies up the middle of, EAST edge 100 m away and WEST edge
        # 150 m away (comfortably past the 120 m west aircraft-contact offset below, so the
        # contact itself is never outside the fence): far enough on both sides that DAA's own
        # fence avoidance never triggers (DAA_MARGIN_FENCE 20 below), close enough on both
        # that the 400 m loiter point falls outside the fence whichever side is tried. North
        # edge well beyond the leg + lookahead.
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                self.offset_location_ne(home, -500, -150),
                self.offset_location_ne(home, -500, 100),
                self.offset_location_ne(home, 8000, 100),
                self.offset_location_ne(home, 8000, -150),
            ],
        )])

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.set_parameters({
            "DAA_MARGIN_CA": 50,
            "DAA_MARGIN_CA_Z": 30,
            "DAA_AVD_ALT": 50,      # non-zero: the loiter is genuinely wanted
            "DAA_AVD_ALT_TP": 1,    # above home - keep the terrain frame out of this
            "DAA_MARGIN_FENCE": 20, # so the 100 m wall is never avoided
        })

        icao = 0xA5A5A9

        def inject_aircraft():
            # 120 m WEST, i.e. inside the corridor, so the contact itself is not outside
            here = self.get_location()
            contact = self.offset_location_ne(here, 0, -120)
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(contact.lat * 1e7),
                int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 10 * 1000),
                0, 0, 0,
                "GAJET05".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,   # crewed -> loiter path
                1, 65535, 1200,
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.do_fence_enable()
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # Stop as soon as a refusal is seen, not after a fixed 60s. Both walls are close
        # enough that the turn-around at WP2 (which swaps which side is "abeam right")
        # cannot open up a legitimately-clear side the way the single-wall version of this
        # corridor could.
        tstart = self.get_sim_time()
        refused = False
        while self.get_sim_time() - tstart < 60 and not refused:
            inject_aircraft()
            self.wait_heartbeat()
            # self.start()'s own fence-clearance check (see PlaneDAALoiterFencePicksClearSide)
            # now catches this before Plane's DO_REPOSITION sanitize is even reached - but
            # either refusal exercises the same no-state-corruption invariant this test exists
            # to prove, so accept whichever one actually fires.
            refused = (self.statustext_in_collections("set_vehicle FAILED") is not None
                       or self.statustext_in_collections("no fence-clear side") is not None)

        announced = self.statustext_in_collections("LOITER AIRCRAFT") is not None
        avoided = self.statustext_in_collections("AVOIDING") is not None
        self.progress("loiter refused=%s, announced=%s, (fence avoidance seen=%s)"
                      % (refused, announced, avoided))

        if not refused:
            raise NotAchievedException(
                "the loiter was never refused, so this test cannot tell the two behaviours "
                "apart - the out-of-fence loiter point did not reproduce")
        if announced:
            raise NotAchievedException(
                "announced LOITER AIRCRAFT although the loiter was refused: the state "
                "machine is claiming to loiter when loiteralt.start() failed")
        self.disarm_vehicle(force=True)
