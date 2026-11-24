'''
Plane autotest: PlaneDAATrapCauseClassified

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAATrapCauseClassified(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAATrapCauseClassified(self):
        '''The trapped-failsafe must classify itself from what actually caused the
        compromise, not from whatever the bendy ruler happens to be closest to.

        daa_compromised_now() fires either on a fence breach or on a real aircraft inside
        the near-miss volume, but trap_update() used to decide "moving" vs "fence" from
        obstacle_avoiding - a different quantity entirely.  An aircraft near-miss could
        therefore be filed as a static fence trap, which is held until the pilot
        intervenes instead of auto-clearing after DAA_TRAP_CLR_S.

        The two are separated here by altitude: the contact is placed above AVD_WCLR_Z,
        so distance_to_obstacle() rejects it on the vertical band and the bendy ruler
        never sees it (obstacle_avoiding stays nil), but still inside AVD_NMAC_Z and
        within the find_aircraft() vertical lookahead of AVD_WCLR_Z + DAA_MARGIN_CA_Z, so
        it is a genuine near-miss.  No fences are loaded at all, so with the old code
        obstacle_avoiding is nil and the trap reports "(fence)"; the cause is an aircraft
        and it must report "(moving)".'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,      # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "FENCE_ENABLE": 0,   # no fences: obstacle_avoiding must stay nil
            # horizontal: 100 m abeam is inside NMAC_XY and inside the find_aircraft
            # lookahead of WCLR_XY + DAA_MARGIN_CA (250 m)
            "AVD_WCLR_XY": 200,
            "AVD_NMAC_XY": 200,
            # vertical: 50 m up is ABOVE WCLR_Z (30, so the bendy ruler drops it) but
            # inside NMAC_Z (80) and inside WCLR_Z + DAA_MARGIN_CA_Z (90)
            "AVD_WCLR_Z": 30,
            "AVD_NMAC_Z": 80,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.set_parameters({
            "DAA_MARGIN_CA": 50,
            "DAA_MARGIN_CA_Z": 60,
            "DAA_AVD_ALT": 0,
            "DAA_TRAP_ACT": 1,    # RTL
            # Fire on the first near-miss cycle.  update() calls trap_update() before
            # DAA.avoid(), so a zero dwell means the trap is decided before the aircraft
            # loiter can start.  DAA_AVD_ALT=0 above already suppresses the loiter
            # entirely (it is only started if DAA_AVD_ALT > 0), so the zero dwell here is
            # belt-and-braces against the loiter's own mode change perturbing the trap.
            "DAA_TRAP_S": 0,
        })

        icao = 0xA5A5A7

        def inject_aircraft():
            here = self.get_location()
            contact = self.offset_location_ne(here, 0, 100)
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(contact.lat * 1e7),
                int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 50 * 1000),
                0, 0, 0,
                "GAJET03".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,   # crewed -> find_aircraft path
                1, 65535, 1200,
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # hold the near-miss until the trap fires (DAA_TRAP_S = 5 s of sustained compromise)
        # Read the trap message out of the STATUSTEXT collection rather than off the wire.
        # A recv_match() loop interleaved with the injection drops messages, and a correctly
        # classified "(fence)" trap is STICKY, so it is announced exactly once and a single
        # missed sample looks identical to the trap never firing at all.
        tstart = self.get_sim_time()
        trapped_text = None
        while self.get_sim_time() - tstart < 90:
            inject_aircraft()
            self.wait_heartbeat()
            for st in self.context_collection('STATUSTEXT'):
                if "TRAPPED" in st.text:
                    trapped_text = st.text.strip()
                    break
            if trapped_text is not None:
                break
        self.set_parameter("DAA_TRAP_ACT", 0)   # stand the failsafe down before cleanup
        if trapped_text is None:
            raise NotAchievedException(
                "the trapped-failsafe never fired for a sustained aircraft near-miss")

        self.progress("trap message: %s" % trapped_text)
        if "(fence)" in trapped_text:
            raise NotAchievedException(
                "aircraft near-miss trap classified as a static fence trap (%s): the cause "
                "is being read from obstacle_avoiding rather than from the compromise"
                % trapped_text)
        if "(moving)" not in trapped_text:
            raise NotAchievedException("unexpected trap message: %s" % trapped_text)
        self.disarm_vehicle(force=True)
