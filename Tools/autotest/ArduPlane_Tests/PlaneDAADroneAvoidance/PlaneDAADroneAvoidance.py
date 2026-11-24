'''
Plane autotest: PlaneDAADroneAvoidance

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAADroneAvoidance(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAADroneAvoidance(self):
        '''planedaa must avoid an ADS-B drone (emitter type UAV = 14).  A drone is
        injected on the path to a waypoint 2 km north.  Unlike a crude aircraft it
        is classified UNCREWED_AIRCRAFT and gets the smaller DAA_MARGIN_UAV standoff with a
        bendy-ruler detour (no loiter-to-altitude), and it is labelled by its ICAO
        in hex rather than a decimal SYSID.  The ADSB_VEHICLE is re-sent
        continuously because AP_Avoidance prunes obstacles after 5 s.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,    # required for AP_Avoidance to pull ADSB samples
            # pin the UAV avoidance envelope so the test is independent of the
            # defaults: horizontal radius and the vertical gate the intruder
            # (injected +10 m) must sit inside
            "AVD_UAV_XY": 150,
            "AVD_UAV_Z": 25,
        })

        # collect before reboot so the script's start-up announcement is captured
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        home = self.home_position_as_location()
        # the drone sits on the northbound leg, ~1 km ahead of home, directly on
        # the path so it is a guaranteed threat once the plane turns towards WP2
        drone_loc = self.offset_location_ne(home, 1000, 0)
        icao = 0xF00080

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)

        # planedaa announces itself once its STARTUP_DELAY has elapsed
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # inject the drone repeatedly (obstacles prune after 5 s); keep its
        # altitude within the ~25 m UAV vertical gate by matching ours.
        tstart = self.get_sim_time()
        avoided = False
        sample_count = [0]
        while self.get_sim_time() - tstart < 120:
            here = self.get_location()
            sample_count[0] += 1
            # see PlaneDAAParkedAircraftIgnored's inject() for why this jitters
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(drone_loc.lat * 1e7) + (sample_count[0] % 2),
                int(drone_loc.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 10000),   # 10 m up, well inside the 25 m gate
                0,      # heading cdeg
                0,      # horizontal velocity cm/s
                0,      # vertical velocity cm/s
                "SIMTL80".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_UAV,   # emitter 14 -> UNCREWED_AIRCRAFT
                1,      # time since last communication
                65535,  # flags
                1200,   # squawk
            )
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if m is not None and "AVOIDING" in m.text:
                avoided = True
                break
        if not avoided:
            raise NotAchievedException("planedaa did not avoid the ADS-B drone")

        # the label must identify it as an ADS-B drone by ICAO (hex), not a
        # decimal SYSID formatted from the 24-bit ICAO address
        self.wait_text("Drone:%06X" % (icao & 0xFFFFFF),
                       check_context=True, timeout=30)

        # an ADS-B-sourced drone (ICAO src_id) must be typed "adsbdrone",
        # not "mavdrone" which is reserved for a real MAVLink system id
        self.wait_text("adsbdrone", check_context=True, timeout=30)

        # and after detouring the plane must still reach the waypoint (not trapped)
        self.wait_current_waypoint(3, timeout=400)
        self.disarm_vehicle(force=True)
