'''
Plane autotest: PlaneDAAStaleAdsbNotExcluded

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAStaleAdsbNotExcluded(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAStaleAdsbNotExcluded(self):
        '''Negative control for PlaneDAAStaleLocationExcluded's source restriction:
        an ADS-B contact (not direct-MAVLink) whose position never changes must still
        be avoided normally - AVD_LOC_STALE_S must not apply to it. ADS-B's own
        timestamp (tslc) is back-dated at arrival, which the staleness check cannot
        tell apart from a genuinely frozen feed, and excluding it would also wrongly
        catch POINT_OBSTACLE, which is deliberately stationary by design. Same
        geometry as PlaneDAAStaleLocationExcluded, but the reaction MUST occur here
        (same assertion style as PlaneDAADroneAvoidance).'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,
            "AVD_ENABLE": 1,
            "AVD_UAV_XY": 150,
            "AVD_UAV_Z": 25,
            "AVD_LOC_STALE_S": 2,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        home = self.home_position_as_location()
        drone_loc = self.offset_location_ne(home, 1000, 0)
        icao = 0xF00082

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        tstart = self.get_sim_time()
        avoided = False
        while self.get_sim_time() - tstart < 120:
            # deliberately NOT jittered, unlike PlaneDAADroneAvoidance - the point
            # here is an unchanging ADS-B feed, with tslc=2 so a sample is already
            # back-dated past AVD_LOC_STALE_S by the time it arrives.
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(drone_loc.lat * 1e7),
                int(drone_loc.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(home.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 90000),
                0, 0, 0,
                "STALEUAV".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_UAV,
                2, 65535, 1200,
            )
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if m is not None and "AVOIDING" in m.text:
                avoided = True
                break
        if not avoided:
            raise NotAchievedException("planedaa did not avoid the unchanging ADS-B drone")

        self.wait_current_waypoint(3, timeout=400)
        self.disarm_vehicle(force=True)
