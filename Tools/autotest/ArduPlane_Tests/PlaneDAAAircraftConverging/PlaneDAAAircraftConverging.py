'''
Plane autotest: PlaneDAAAircraftConverging

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAAircraftConverging(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAAircraftConverging(self):
        '''planedaa must AVOID a crewed aircraft that is in the outer well-clear band but
        converging, and commit the loiter-to-altitude descent.  This is the mirror of
        PlaneDAAAircraftCpaGate: the same outer-band geometry (contact at 230 m, beyond
        the 200 m AVD_WCLR_XY well-clear radius but inside the 250 m detection band), but
        the contact is closing (heading at the vehicle) rather than opening, so the CPA
        gate must treat it as a conflict.  Three things are proven that no other test
        covers together: (1) a converging contact in the outer band engages the loiter
        (conflict via closing/CPA, not mere proximity); (2) the loiter-to-altitude path
        actually DESCENDS the vehicle toward DAA_AVD_ALT (the vertical gap that resolves
        a crewed-aircraft overflight - the real CF-DQF/log_102 encounter); (3) once the contact stops
        (prunes after 5 s, dwell expires) the vehicle cleanly resumes AUTO.  The
        ADSB_VEHICLE is re-sent each cycle because AP_Avoidance prunes obstacles after
        5 s.'''
        self.install_planedaa_scripts()
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,         # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,        # required for AP_Avoidance to pull ADSB samples
            "AVD_WCLR_XY": 200,     # well-clear radius; detection band is 200..250 m
            "AVD_WCLR_Z": 100,      # generous vertical gate so altitude never gates detection
        })
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameters({
            "DAA_MARGIN_CA": 50,    # detection = AVD_WCLR_XY + this = 250 m
            "DAA_MARGIN_CA_Z": 50,
            "DAA_AVD_ALT": 40,      # loiter-to-altitude descend target ...
            "DAA_AVD_ALT_TP": 1,    # ... 40 m above home (deterministic; below the 100 m cruise)
            "DAA_CPA_MIN": 2,       # closing-speed (m/s) threshold for "is it closing?"
        })

        icao = 0xD1D1D1
        cruise_alt_m = 100

        def inject_converging_plane():
            # mirror of PlaneDAAAircraftCpaGate: 230 m east (outer band, beyond the 200 m
            # well-clear radius), but heading WEST (at the vehicle) at 40 m/s, matched
            # altitude -> a closing contact whose CPA falls inside well-clear = conflict.
            here = self.get_location()
            contact = self.offset_location_ne(here, 0, 230)
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(contact.lat * 1e7),
                int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000),           # matched altitude -> inside the vertical gate
                27000,                           # heading 270 deg = due west (toward the vehicle)
                4000,                            # 40 m/s horizontal velocity -> closing the range
                0,
                "CONVRG1".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,   # is_adsb_aircraft -> loiter path
                1, 65535, 1200,
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 0, cruise_alt_m),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)
        # settle at cruise before injecting: reach cruise, then let it level off, so the
        # loiter engages from steady flight rather than mid-climb.  A mid-climb engage
        # overshoots up past cruise and eats into the descend budget (the flaky case).
        self.wait_altitude(cruise_alt_m - 5, cruise_alt_m + 5, relative=True, timeout=120)
        self.delay_sim_time(8, reason="level off at cruise before the converging contact")

        # --- engage: a converging contact in the outer band must trigger the loiter
        #     (conflict via CPA/closing, not proximity - it is beyond well-clear).
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 45:
            inject_converging_plane()
            if self.mav.flightmode == "GUIDED":
                break
            self.wait_heartbeat()
        if self.mav.flightmode != "GUIDED":
            raise NotAchievedException(
                "did not engage loiter (GUIDED) for a converging crewed aircraft in the outer "
                "well-clear band - CPA gate should treat a closing contact as a conflict")
        engage_alt_m = self.get_altitude(relative=True)

        # --- descend: while the conflict persists the loiter-to-altitude path must
        #     drive the vehicle down toward DAA_AVD_ALT (40 m). Prove it sheds a clear
        #     vertical gap (>= 25 m below the engage altitude) - the manoeuvre that
        #     actually resolves a crewed-aircraft overflight.
        descend_floor_m = engage_alt_m - 25
        tstart = self.get_sim_time()
        descended = False
        while self.get_sim_time() - tstart < 90:
            inject_converging_plane()
            self.wait_heartbeat()
            if self.get_altitude(relative=True) < descend_floor_m:
                descended = True
                break
        if not descended:
            raise NotAchievedException(
                "loiter engaged but the vehicle did not descend toward DAA_AVD_ALT "
                "(still above %.0f m after 90 s; engaged at %.0f m)" % (descend_floor_m, engage_alt_m))

        # --- resume: stop injecting; the contact prunes (5 s) and the dwell expires,
        #     so the vehicle must cleanly return to AUTO and continue the mission.
        self.wait_mode("AUTO", timeout=30)
        self.disarm_vehicle(force=True)
