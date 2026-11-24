'''
Plane autotest: PlaneDAAMaskedDroneBehindDismissedAircraft

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAMaskedDroneBehindDismissedAircraft(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAMaskedDroneBehindDismissedAircraft(self):
        '''A crewed aircraft's wide well-clear radius can mask a drone well behind it in
        AP_Avoidance::distance_to_obstacle()'s single-winner search (bigger radius wins
        the effective-distance comparison - raw range minus radius - even though the
        drone is what genuinely still needs avoiding once the aircraft itself stops
        being a conflict).

        validate_horizontal_release()'s masked-contact re-probe used to project along
        candidate_bearing_deg - the sweep's own possibly-still-deviated bearing from
        avoiding the just-dismissed aircraft - not the real direct bearing to the
        target. A probe along the deviated bearing could read "clear" while the real
        resumed leg, never actually probed, was not - confirmed live via DAAR's FnlB
        field reading 12-48 deg off the drone's true bearing in earlier attempts at this
        test. Fixed by probing the real direct path instead.

        Reproduces exactly that: a crewed aircraft 300 m ahead (closing, so its wide
        radius wins and masks the drone, which is still 600+ m away at this point)
        engages avoidance; it is then switched to strongly opening (dismissed as
        non-conflicting). A genuinely-closing drone fixed 900 m from home - within
        DAA_DETECT_M (1000 m default) - must then be adopted instead of releasing.

        Asserted via the dataflash DAAR log (ObjT==UNCREWED_AIRCRAFT with Gon==1), not GCS
        text: the general bendy-ruler path never changes flight mode (unlike the
        loiter-to-altitude path, it steers within AUTO), and the "AVOIDING"/"ALERT"
        announcements are both throttled (5 s) from the aircraft's own announcement
        moments earlier, so the drone's adoption can be real but never get a chance to
        print before this short episode resolves - confirmed happening exactly that way
        in an earlier attempt at this test (DAAR showed ObjT flip to UNCREWED_AIRCRAFT for
        ~1.7s with no matching STATUSTEXT ever appearing).'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "AVD_WCLR_XY": 150,     # aircraft masking radius
            "AVD_WCLR_Z": 50,
            "AVD_UAV_XY": 80,       # drone radius - much smaller, the thing being masked
            "AVD_UAV_Z": 25,
        })
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameters({
            # aircraft standoff = 160 m - small relative to its 300 m range so
            # avoiding it needs almost no detour
            "DAA_MARGIN_CA": 10,
            "DAA_MARGIN_UAV": 20,   # drone standoff = 100 m
        })

        home = self.home_position_as_location()
        drone_loc = self.offset_location_ne(home, 900, 0)
        aircraft_icao = 0xA1A1A1
        drone_icao = 0xD2D2D2
        drone_sample_count = [0]

        def inject_aircraft(closing):
            # kept a fixed 300 m ahead of wherever the vehicle is right now (same
            # technique as PlaneDAAAircraftConverging) - only the reported heading/speed
            # fields drive assess_obstacle_motion()'s closing/opening classification.
            here = self.get_location()
            contact = self.offset_location_ne(here, 300, 0)
            heading_cdeg = 18000 if closing else 0     # 180=toward us, 0=away/ahead
            speed_cms = 3000 if closing else 4000
            self.mav.mav.adsb_vehicle_send(
                aircraft_icao,
                int(contact.lat * 1e7),
                int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000),
                heading_cdeg, speed_cms, 0,
                "MASKAC1".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,   # is_adsb_aircraft -> AVD_WCLR_XY radius
                1, 65535, 1200,
            )

        def inject_drone():
            # fixed position, zero velocity: genuinely closing as the vehicle itself
            # flies toward it (same technique as PlaneDAADroneAvoidance). Jittered -
            # see PlaneDAAParkedAircraftIgnored's inject() for why.
            here = self.get_location()
            drone_sample_count[0] += 1
            self.mav.mav.adsb_vehicle_send(
                drone_icao,
                int(drone_loc.lat * 1e7) + (drone_sample_count[0] % 2),
                int(drone_loc.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 10000),
                0, 0, 0,
                "MASKDR1".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_UAV,     # -> AVD_UAV_XY radius
                1, 65535, 1200,
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # --- engage: the closing aircraft must win (mask) and be announced
        aircraft_label = "%06X" % (aircraft_icao & 0xFFFFFF)
        tstart = self.get_sim_time()
        engaged = False
        while self.get_sim_time() - tstart < 60 and not engaged:
            inject_aircraft(closing=True)
            inject_drone()
            self.wait_heartbeat()
            engaged = self.statustext_in_collections(aircraft_label) is not None
        if not engaged:
            raise NotAchievedException(
                "the masking aircraft was never avoided - geometry did not reproduce")

        # --- dismiss the aircraft (now strongly opening) while the drone keeps closing.
        # A fixed dwell, not an early-exit on GCS text or flight mode: see the
        # docstring for why neither is a reliable signal here.
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 20:
            inject_aircraft(closing=False)
            inject_drone()
            self.wait_heartbeat()

        log_filepath = self.current_onboard_log_filepath()
        self.disarm_vehicle(force=True)

        self.progress("Inspecting DFReader for the masked drone's adoption (DAAR ObjT)")
        dfreader = self.dfreader_for_path(log_filepath)
        OBSTACLE_TYPE_UNCREWED_AIRCRAFT = 1
        adopted_drone = False
        while True:
            m = dfreader.recv_match(type=['DAAR'])
            if m is None:
                break
            if m.ObjT == OBSTACLE_TYPE_UNCREWED_AIRCRAFT and m.Gon == 1:
                adopted_drone = True
                break
        if not adopted_drone:
            raise NotAchievedException(
                "never saw DAAR.ObjT==UNCREWED_AIRCRAFT with Gon==1 after the aircraft was "
                "dismissed - the masked-contact re-probe did not find/adopt the drone")
