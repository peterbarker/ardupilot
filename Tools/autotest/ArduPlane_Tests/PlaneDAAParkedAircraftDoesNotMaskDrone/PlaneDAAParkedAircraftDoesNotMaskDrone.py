'''
Plane autotest: PlaneDAAParkedAircraftDoesNotMaskDrone

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAParkedAircraftDoesNotMaskDrone(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAParkedAircraftDoesNotMaskDrone(self):
        '''AP_Avoidance's find_threats()/find_aircraft() return a single closest obstacle,
        compared by clearance to each obstacle's OWN keep-out radius (AVD_WCLR_XY for a
        crewed aircraft, AVD_UAV_XY for a drone) - not raw range.  A crewed aircraft parked
        near home, with its much larger AVD_WCLR_XY radius, can therefore win that search
        and get excluded as parked (AVD_GND_ALT/AVD_GND_SPD) EVERY cycle - masking a real,
        much closer drone that never gets a turn to be evaluated at all.  This reproduces
        that exact scenario found live in the AreaXO demo (a stationary "Niska" stand-in
        near home hid a hovering drone that passed within a few metres of the flight path)
        and checks the drone still gets avoided.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,    # required for AP_Avoidance to pull ADSB samples
            "AVD_UAV_XY": 150,
            "AVD_UAV_Z": 25,
            "AVD_WCLR_XY": 600,  # exaggerate the radius gap so a masking regression is unambiguous
            "AVD_GND_ALT": 3,
            "AVD_GND_SPD": 2,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        home = self.home_position_as_location()
        # parked "crewed aircraft" sits 30 m from home, at home's own altitude, stationary -
        # squarely inside AVD_GND_ALT/AVD_GND_SPD, and well within its own 600 m AVD_WCLR_XY
        # radius for the whole flight
        parked_loc = self.offset_location_ne(home, 30, 0)
        parked_icao = 0xF00090
        # the drone sits on the northbound leg, ~1 km ahead, directly on the path - same
        # geometry as PlaneDAADroneAvoidance
        drone_loc = self.offset_location_ne(home, 1000, 0)
        drone_icao = 0xF00080

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        tstart = self.get_sim_time()
        avoided = False
        sample_count = [0]
        while self.get_sim_time() - tstart < 120:
            here = self.get_location()
            sample_count[0] += 1
            # see PlaneDAAParkedAircraftIgnored's inject() for why this jitters
            self.mav.mav.adsb_vehicle_send(
                parked_icao,
                int(parked_loc.lat * 1e7) + (sample_count[0] % 2),
                int(parked_loc.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(home.get_alt_m(AltFrame.ABSOLUTE) * 1000),
                0, 0, 0,       # heading, hor/vert velocity: stationary
                "PARKED01".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_SMALL,   # emitter 2 -> CREWED_AIRCRAFT
                1, 65535, 1200,
            )
            self.mav.mav.adsb_vehicle_send(
                drone_icao,
                int(drone_loc.lat * 1e7) + (sample_count[0] % 2),
                int(drone_loc.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 10000),   # 10 m up, inside the 25 m gate
                0, 0, 0,
                "SIMTL81".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_UAV,     # emitter 14 -> UNCREWED_AIRCRAFT
                1, 65535, 1200,
            )
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            # must specifically be avoiding the DRONE by its ICAO label - the parked aircraft
            # is also nearby and, unfixed, can itself get treated as a real threat (its own
            # "AVOIDING: PARKED01" would otherwise pass this check for the wrong reason)
            if m is not None and ("AVOIDING: Drone:%06X" % (drone_icao & 0xFFFFFF)) in m.text:
                avoided = True
                break
        if not avoided:
            raise NotAchievedException(
                "planedaa did not avoid the drone - masked by the parked aircraft?")

        self.wait_current_waypoint(3, timeout=400)
        self.disarm_vehicle(force=True)
