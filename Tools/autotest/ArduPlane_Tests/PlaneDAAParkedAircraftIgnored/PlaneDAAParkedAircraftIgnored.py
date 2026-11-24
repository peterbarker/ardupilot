'''
Plane autotest: PlaneDAAParkedAircraftIgnored

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAParkedAircraftIgnored(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAParkedAircraftIgnored(self):
        '''A crewed aircraft parked near home (within AVD_GND_ALT/AVD_GND_SPD) must never
        itself trigger avoidance. This is the original bug found live in the AreaXO demo:
        an unfixed vehicle read a stationary aircraft near the runway/pad as an airborne
        near-miss and got stuck alternating between landing and RTL/loiter, never completing
        a takeoff or landing. No second obstacle is injected here - this proves only the
        exclusion itself, independent of PlaneDAAParkedAircraftDoesNotMaskDrone's masking
        scenario, which needs a real drone in the picture too.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,
            "AVD_ENABLE": 1,
            "AVD_GND_ALT": 3,
            "AVD_GND_SPD": 2,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameter("DAA_TRAP_ACT", 1)  # non-default, so the "TRAPPED" check below has teeth

        home = self.home_position_as_location()
        parked_loc = self.offset_location_ne(home, 30, 0)
        parked_icao = 0xF00091

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        sample_count = [0]

        def inject():
            # A real GPS/EKF-sourced position always carries sample-to-sample jitter - an
            # exactly-repeating one looks like a frozen feed to AVD_LOC_STALE_S, which is
            # not what this test means to exercise. Alternating +0/+1 units of 1e-7 deg
            # (~1.1 cm, negligible for geometry) guarantees consecutive samples never match.
            sample_count[0] += 1
            self.mav.mav.adsb_vehicle_send(
                parked_icao,
                int(parked_loc.lat * 1e7) + (sample_count[0] % 2),
                int(parked_loc.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(home.get_alt_m(AltFrame.ABSOLUTE) * 1000),
                0, 0, 0,       # heading, hor/vert velocity: stationary
                "PARKED02".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_SMALL,   # emitter 2 -> CREWED_AIRCRAFT
                1, 65535, 1200,
            )

        self.daa_watch_no_reaction(60, [
            "AVOIDING", "LOITERING", "LOITER AIRCRAFT", "TRAPPED",
            "Near Miss", "Loss of Well Clear",
        ], inject)

        self.wait_current_waypoint(3, timeout=400)
        self.disarm_vehicle(force=True)
