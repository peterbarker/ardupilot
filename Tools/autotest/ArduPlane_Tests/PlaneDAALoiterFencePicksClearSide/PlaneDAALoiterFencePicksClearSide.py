'''
Plane autotest: PlaneDAALoiterFencePicksClearSide

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAALoiterFencePicksClearSide(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAALoiterFencePicksClearSide(self):
        '''When only ONE side of the aircraft loiter is fence-blocked, it must pick the
        other side rather than refuse - this is PlaneDAALoiterFailNoState's original
        geometry (a single close wall, abeam the default right-hand loiter), kept as its
        own test because the fix that made it pass changed what that test could prove.

        self.start() places the loiter centre WP_LOITER_RAD abeam, tries the requested side
        first, and - new behaviour - checks the real fence clearance of the whole orbit
        (radius_m + DAA_MARGIN_FENCE), not just Plane's own DO_REPOSITION sanitize check on
        the centre point alone. If that side is blocked it tries the other side once before
        giving up. Confirmed live this matters: AreaXO 2026-10-06, a loiter orbiting toward
        a crewed aircraft breached a cell-tower exclusion fence nothing had checked the
        orbit against.

        Only the EAST wall is close here (100 m); the west side is wide open, so the flip
        must succeed, loiter LEFT, and keep loitering - never refuse.'''
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
        # Only the EAST wall is close (100 m); the west side is wide open (3000 m), so the
        # default right-hand loiter point falls outside the fence but the flipped left-hand
        # one does not.
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                self.offset_location_ne(home, -500, -3000),
                self.offset_location_ne(home, -500, 100),
                self.offset_location_ne(home, 8000, 100),
                self.offset_location_ne(home, 8000, -3000),
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

        icao = 0xA5A5AA

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
                "GAJET06".encode("ascii"),
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

        # Stop as soon as the loiter is actually running, not after a fixed 60s - same
        # turn-around caveat as PlaneDAALoiterFailNoState: this only holds outbound.
        tstart = self.get_sim_time()
        loitering = False
        while self.get_sim_time() - tstart < 60 and not loitering:
            inject_aircraft()
            self.wait_heartbeat()
            loitering = self.statustext_in_collections("LOITERING to") is not None

        flipped_left = self.statustext_in_collections("LOITER left") is not None
        refused = self.statustext_in_collections("set_vehicle FAILED") is not None
        no_clear_side = self.statustext_in_collections("no fence-clear side") is not None
        self.progress("loitering=%s, flipped_left=%s, refused=%s, no_clear_side=%s"
                      % (loitering, flipped_left, refused, no_clear_side))

        if not loitering:
            raise NotAchievedException(
                "the loiter never started - the fence-clear side was not found, or the "
                "east-wall-only geometry did not reproduce")
        if not flipped_left:
            raise NotAchievedException(
                "loitering started but never announced LOITER left - did it pick the "
                "blocked right side instead of flipping?")
        if refused or no_clear_side:
            raise NotAchievedException(
                "a refusal message was seen even though the loiter is running - the two "
                "code paths disagree with each other")
        self.disarm_vehicle(force=True)
