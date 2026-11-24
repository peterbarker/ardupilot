'''
Plane autotest: PlaneDAAParkedDroneIgnored

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAParkedDroneIgnored(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAParkedDroneIgnored(self):
        '''As PlaneDAAParkedAircraftIgnored, but for a MAVLink drone (emitter type UAV)
        parked near home rather than a crewed aircraft - is_parked() must apply to both
        traffic types. This exercises AP_Avoidance::distance_to_obstacle (the general
        bendy-ruler path), not just the crewed-aircraft-only find_aircraft() path
        PlaneDAAParkedAircraftIgnored exercises.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,
            "AVD_ENABLE": 1,
            "AVD_UAV_XY": 150,
            "AVD_UAV_Z": 25,
            "AVD_GND_SPD": 2,
        })

        self.assert_parameter_value_pct("AVD_GND_ALT", 10, 0)

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameter("DAA_TRAP_ACT", 1)  # non-default, so the "TRAPPED" check below has teeth

        home = self.home_position_as_location()
        parked_loc = self.offset_location_ne(home, 30, 0)
        parked_icao = 0xF00092

        # AVD_UAV_Z (25 m) is far tighter than a crewed aircraft's well-clear-Z (~76 m
        # default), so keep the whole mission inside 20 m - a climb through 50-80 m like
        # PlaneDAAParkedAircraftIgnored's would only be inside the drone's vertical gate for
        # a few seconds after liftoff, making a regression here pass or fail by luck rather
        # than by design.
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        sample_count = [0]

        def inject():
            # see PlaneDAAParkedAircraftIgnored's inject() for why this jitters
            sample_count[0] += 1
            self.mav.mav.adsb_vehicle_send(
                parked_icao,
                int(parked_loc.lat * 1e7) + (sample_count[0] % 2),
                int(parked_loc.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                # Parked contact with an 8 m altitude-reference error, inside the default 10 m gate.
                int((home.get_alt_m(AltFrame.ABSOLUTE) + 8) * 1000),
                0, 0, 0,       # heading, hor/vert velocity: stationary
                "PARKED03".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_UAV,     # emitter 14 -> UNCREWED_AIRCRAFT
                1, 65535, 1200,
            )

        self.daa_watch_no_reaction(60, [
            "AVOIDING", "LOITERING", "LOITER AIRCRAFT", "TRAPPED",
            "Near Miss", "Loss of Well Clear",
        ], inject)

        self.wait_current_waypoint(3, timeout=400)
        self.disarm_vehicle(force=True)
