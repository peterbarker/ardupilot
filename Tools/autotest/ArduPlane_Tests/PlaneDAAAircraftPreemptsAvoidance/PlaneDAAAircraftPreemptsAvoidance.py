'''
Plane autotest: PlaneDAAAircraftPreemptsAvoidance

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAAircraftPreemptsAvoidance(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAAircraftPreemptsAvoidance(self):
        '''Crewed traffic must be able to take over from an avoidance already in progress.

        DAA.avoid() decides what to do each cycle through an if/elseif chain.  The
        mid-manoeuvre branch - current_state is hovering, avoiding or landing - existed so
        the vehicle would not re-decide halfway through a manoeuvre, but it was tested
        BEFORE the crewed-aircraft loiter trigger.  So once the plane was avoiding a fence
        or a drone, an aircraft appearing beside it populated aircraft_avoiding and raised
        the ALERT, but the loiter takeover below was never reached: the highest-priority
        threat was suppressed by a lower-priority one already being handled.

        A 400 m exclusion circle is placed across the mission leg so fence avoidance is
        running and stays running, and only once "AVOIDING" has been seen is a steady
        crewed aircraft (emitter LIGHT) injected 120 m abeam - inside AVD_WCLR_XY, so
        assess_obstacle_motion() calls it a conflict unconditionally.  The loiter must
        start.  On the unfixed chain it never does.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "AVD_WCLR_XY": 200,
            "AVD_WCLR_Z": 50,
            "FENCE_ENABLE": 0,  # enabled in flight so arming is unimpeded
            "FENCE_ACTION": 0,  # report only - a breach must not trigger an RTL
            "FENCE_TYPE": 4,    # polygon/circle fences
        })

        home = self.home_position_as_location()
        # big enough that the detour lasts long enough to inject traffic mid-avoidance
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            {"radius": 400, "loc": self.offset_location_ne(home, 1400, 0)},
        )])

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.set_parameters({
            "DAA_MARGIN_CA": 50,
            "DAA_MARGIN_CA_Z": 30,
            "DAA_AVD_ALT": 50,      # non-zero: the loiter is genuinely wanted
            "DAA_AVD_ALT_TP": 1,    # above home - keep the terrain frame out of this
            "DAA_MARGIN_FENCE": 50,
        })

        icao = 0xA5A5AA

        def inject_aircraft():
            # 120 m abeam of wherever the vehicle is now: inside AVD_WCLR_XY (200 m), so it
            # is a conflict however it is moving, and co-altitude so the vertical gate passes
            here = self.get_location()
            contact = self.offset_location_ne(here, 0, 120)
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(contact.lat * 1e7),
                int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000),
                0, 0, 0,
                "GAJET07".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,   # crewed -> loiter path
                1, 65535, 1200,
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2600, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.do_fence_enable()
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # wait until fence avoidance is genuinely running before introducing the aircraft
        self.wait_text("AVOIDING", check_context=True, timeout=180)
        self.progress("fence avoidance active - now injecting crewed traffic")

        loitered = False
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 60:
            inject_aircraft()
            self.wait_heartbeat()
            if self.statustext_in_collections("LOITER AIRCRAFT") is not None:
                loitered = True
                break

        if not loitered:
            raise NotAchievedException(
                "a crewed aircraft inside well-clear did not start the loiter while a fence "
                "avoidance was in progress: the mid-manoeuvre branch is still suppressing "
                "the higher-priority traffic takeover")
        self.progress("crewed traffic preempted the fence avoidance")
        self.disarm_vehicle(force=True)
