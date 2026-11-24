'''
Plane autotest: PlaneDAADroneCpaGate

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAADroneCpaGate(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAADroneCpaGate(self):
        '''The type-aware CPA standoff must let the drone bendy-ruler IGNORE a drone that is
        ahead on the path but clearly diverging.  A drone is injected ~150 m dead ahead moving
        directly away faster than the vehicle, so its closest approach stays well beyond the
        drone standoff (AVD_UAV_XY) and it is opening: no avoidance manoeuvre is needed.  The
        vehicle must reach the waypoint without ever announcing "AVOIDING" for the drone.  Fails
        on the pre-change code, whose conflict test used the (much larger) aircraft well-clear
        radius for every type, so it avoided the diverging drone.'''
        self.install_planedaa_scripts()
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "AVD_UAV_XY": 75,       # drone standoff (get_standoff for UNCREWED_AIRCRAFT)
            "AVD_UAV_Z": 50,        # generous so altitude never gates detection here
        })
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameters({
            "DAA_MARGIN_UAV": 10,
            "DAA_CPA_MIN": 2,
        })

        icao = 0xC3C3C3

        def inject_diverging_drone():
            here = self.get_location()
            # 150 m dead ahead (north, on the path), moving further north (away) at 40 m/s
            contact = self.offset_location_ne(here, 150, 0)
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(contact.lat * 1e7),
                int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000),
                0,                               # heading 0 = due north (directly away, ahead)
                4000,                            # 40 m/s -> opens faster than the ~18 m/s cruise
                0,
                "DIVDRN01".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_UAV,   # emitter 14 -> drone bendy-ruler path
                1, 65535, 1200,
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # inject the diverging drone steadily for 25 s; the CPA gate must NOT avoid it
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 25:
            inject_diverging_drone()
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if m is not None and "AVOIDING" in m.text and "Drone" in m.text:
                raise NotAchievedException(
                    "avoided a diverging drone ('%s') - the type-aware CPA standoff should "
                    "have treated it as leaving" % m.text.strip())
        self.disarm_vehicle(force=True)
