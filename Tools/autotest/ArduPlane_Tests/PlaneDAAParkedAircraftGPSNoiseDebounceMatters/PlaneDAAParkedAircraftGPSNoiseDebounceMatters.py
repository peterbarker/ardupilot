'''
Plane autotest: PlaneDAAParkedAircraftGPSNoiseDebounceMatters

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAParkedAircraftGPSNoiseDebounceMatters(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAParkedAircraftGPSNoiseDebounceMatters(self):
        '''Companion to PlaneDAAParkedAircraftGPSNoiseIgnored, proving AVD_GND_DEB is
        actually doing something in that same geometry rather than the spike simply
        never mattering - same contact, same position, same noisy-sample injection,
        AVD_GND_DEB=0 (the prior, single-sample behaviour) instead of 2. A reaction
        must occur: AVD_WCLR_XY (609.6 m default) comfortably covers a contact sitting
        400 m directly ahead, so the very first noisy sample should be read as a
        genuine moving aircraft and avoided - a crewed-aircraft emitter type takes the
        loiter-to-altitude path (ALERT/LOITERING), not the generic bendy-ruler
        "AVOIDING" text, so any of the same texts PlaneDAAParkedAircraftGPSNoiseIgnored
        treats as forbidden here count as the reaction this test wants to see.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,
            "AVD_ENABLE": 1,
            "AVD_GND_ALT": 3,
            "AVD_GND_SPD": 2,
            "AVD_GND_DEB": 0,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        home = self.home_position_as_location()
        parked_loc = self.offset_location_ne(home, 400, 0)
        parked_icao = 0xF00094

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        sample_count = [0]

        def inject():
            sample_count[0] += 1
            noisy = (sample_count[0] % 10 == 0)
            # see PlaneDAAParkedAircraftIgnored's inject() for why this jitters
            self.mav.mav.adsb_vehicle_send(
                parked_icao,
                int(parked_loc.lat * 1e7) + (sample_count[0] % 2),
                int(parked_loc.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(home.get_alt_m(AltFrame.ABSOLUTE) * 1000),
                0,
                500 if noisy else 0,   # 5 m/s horizontal "velocity" on the noisy sample only
                0,
                "PARKED05".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_SMALL,
                1, 65535, 1200,
            )

        reaction_texts = [
            "AVOIDING", "LOITERING", "LOITER AIRCRAFT", "TRAPPED",
            "Near Miss", "Loss of Well Clear",
        ]
        reacted = False
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 60 and not reacted:
            inject()
            self.wait_heartbeat()
            reacted = any(self.statustext_in_collections(text) is not None
                          for text in reaction_texts)
        if not reacted:
            raise NotAchievedException(
                "AVD_GND_DEB=0 never produced a reaction to the noisy parked contact - "
                "this geometry cannot tell whether AVD_GND_DEB does anything")

        self.disarm_vehicle(force=True)
