'''
Plane autotest: PlaneDAAClimbAltitudeSeparation

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAClimbAltitudeSeparation(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAClimbAltitudeSeparation(self):
        '''planedaa's primary bendy-ruler probe (step 1) must use the vehicle's REAL
        altitude, not the candidate's target altitude, when checking vertical separation
        from an obstacle - otherwise a climbing (or descending) leg is flattened to a
        level line and a drone directly ahead at the vehicle's actual, much lower,
        current altitude is invisible to it.

        probe_bearing() builds adjusted_loc (roughly "where the aircraft will be just
        after turning") via location_for_candidate() -> location_after_course_change() ->
        location_project(), which always adopts the CANDIDATE'S TARGET altitude.  test_loc
        (the far end of the step-1 probe) does the same.  So both ends of the step-1
        segment carried the same (target) altitude, discarding the real climb - the
        C++ side (AP_Avoidance::distance_to_obstacle()) interpolates altitude along
        whatever segment it is handed, specifically so a climbing/descending leg is
        represented correctly, but a flattened Lua segment defeats that.

        Reproduces this with a steep, long climb (30 m -> 300 m) and a stationary drone
        close to home, directly on the path, whose altitude is injected to continuously
        TRACK the vehicle's own real altitude (10 m above it) - the same idiom every
        other DAA drone test uses to stay inside the vertical gate regardless of exactly
        when detection happens.  This isolates the bug cleanly: the fixed code always
        sees a ~10 m gap (adjusted_loc carries the real altitude), so it must engage
        avoidance almost immediately, early in the climb.  The unfixed code instead
        computes |target_alt - drone_alt|, which is close to the full 270 m climb for
        as long as the vehicle is still well below the 300 m target - so it can only
        avoid, if at all, once the climb is nearly finished and the vehicle's real
        altitude has coincidentally caught up with the (irrelevant, to it) target.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,    # required for AP_Avoidance to pull ADSB samples
            # generous horizontal radius - this test is about the VERTICAL gate, not a
            # tight lateral margin; the climbing turn-onto-course leaves a little drift
            "AVD_UAV_XY": 300,
            "AVD_UAV_Z": 25,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        home = self.home_position_as_location()
        takeoff_alt_m = 30
        wp_alt_m = 300  # a steep, long climb so the vehicle is still well below this
        # for a good while after takeoff - the gap that exposes the flattened probe
        # planedaa's own STARTUP_DELAY (25 s) means the vehicle can already be several
        # hundred metres down the leg before the script evaluates anything at all - the
        # drone must sit well beyond that so it is not already behind the vehicle by
        # the time DAA starts, whatever this run's exact climb-out timing turns out to be
        drone_loc = self.offset_location_ne(home, 900, 0)  # on the path, early in the climb
        icao = 0xF000A0

        drone_sample_count = [0]

        def inject_drone():
            here = self.get_location()
            drone_sample_count[0] += 1
            # see PlaneDAAParkedAircraftIgnored's inject() for why this jitters
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(drone_loc.lat * 1e7) + (drone_sample_count[0] % 2),
                int(drone_loc.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 10000),  # 10 m above CURRENT altitude
                0, 0, 0,       # heading, hor/vert velocity: stationary
                "CLIMBUAV".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_UAV,   # emitter 14 -> UNCREWED_AIRCRAFT
                1, 65535, 1200,
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, takeoff_alt_m),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 4000, 0, wp_alt_m),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # inject repeatedly (obstacles prune after 5 s) until avoidance engages or the
        # vehicle has climbed almost all the way to the target - by which point the bug
        # and the fix become indistinguishable, so there is nothing more to prove
        drone_label = "Drone:%06X" % (icao & 0xFFFFFF)  # pin the avoidance to this drone
        avoided = False
        avoid_alt_m = None
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 150:
            inject_drone()
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if m is not None and "AVOIDING" in m.text and drone_label in m.text:
                avoided = True
                avoid_alt_m = self.get_altitude(relative=True)
                break
            if self.get_altitude(relative=True) > wp_alt_m - 50:
                break
        if not avoided:
            raise NotAchievedException(
                "planedaa did not avoid a drone tracking its own altitude, directly "
                "ahead during a steep climb - the step-1 probe segment is being "
                "flattened to the candidate's target altitude instead of the "
                "vehicle's real one")
        # must have been caught EARLY in the climb, not coincidentally near the end
        # once the vehicle's real altitude has drifted up to meet the target
        if avoid_alt_m > wp_alt_m - 100:
            raise NotAchievedException(
                "avoidance engaged only at %.0f m, too close to the %.0f m target to "
                "be a real early-climb vertical-separation catch" %
                (avoid_alt_m, wp_alt_m))

        self.disarm_vehicle(force=True)
