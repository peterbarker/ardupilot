'''
Plane autotest: PlaneDAADescentAltitudeSeparation

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAADescentAltitudeSeparation(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAADescentAltitudeSeparation(self):
        '''As PlaneDAAClimbAltitudeSeparation, but descending - the far end of the step-1
        (and step-2) probe segment still adopts the candidate's target altitude outright,
        so a drone parked well short of the target reads as far more vertically separated
        than it really is on a long descent too, not only on a climb.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,
            "AVD_ENABLE": 1,
            "AVD_UAV_XY": 300,
            "AVD_UAV_Z": 25,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        home = self.home_position_as_location()
        takeoff_alt_m = 300
        wp_alt_m = 30
        drone_loc = self.offset_location_ne(home, 900, 0)
        icao = 0xF000A1

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
                "DESCUAV1".encode("ascii"),
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

        drone_label = "Drone:%06X" % (icao & 0xFFFFFF)
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
            if self.get_altitude(relative=True) < wp_alt_m + 50:
                break
        if not avoided:
            raise NotAchievedException(
                "planedaa did not avoid a drone tracking its own altitude, directly "
                "ahead during a steep descent - the far end of the probe segment is "
                "being flattened to the candidate's target altitude instead of "
                "interpolating the real one")
        # must have been caught EARLY in the descent, not coincidentally near the end
        # once the vehicle's real altitude has dropped to meet the target
        if avoid_alt_m < wp_alt_m + 100:
            raise NotAchievedException(
                "avoidance engaged only at %.0f m, too close to the %.0f m target to "
                "be a real early-descent vertical-separation catch" %
                (avoid_alt_m, wp_alt_m))

        self.disarm_vehicle(force=True)
