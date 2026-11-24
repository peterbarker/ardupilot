'''
Plane autotest: PlaneDAAAircraftCpaGate

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAAircraftCpaGate(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAAircraftCpaGate(self):
        '''planedaa's conservative CPA gate must SUPPRESS the aircraft loiter for a
        detected crewed aircraft that is in the outer well-clear band and clearly diverging
        (opening range, closest approach beyond well-clear).  The plane is injected inside
        the detection volume (< AVD_WCLR_XY + DAA_MARGIN_CA) but beyond the AVD_WCLR_XY
        well-clear radius, moving directly away, so it is detected but is not a conflict:
        the vehicle must stay in AUTO and not loiter.  (A close or converging aircraft still
        loiters unconditionally - the conservative floor - proven by
        PlaneDAAAircraftLoiterNoFlip.)  Fails on the pre-CPA-gate code, which loitered for
        any detected aircraft regardless of motion.'''
        self.install_planedaa_scripts()
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "AVD_WCLR_XY": 200,     # well-clear radius; detection band is 200..250 m
            "AVD_WCLR_Z": 100,      # generous vertical gate so altitude never gates detection
        })
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameters({
            "DAA_MARGIN_CA": 50,    # detection = AVD_WCLR_XY + this = 250 m
            "DAA_MARGIN_CA_Z": 50,
            "DAA_AVD_ALT": 50,
            "DAA_CPA_MIN": 2,       # closing-speed (m/s) below which a receding contact is "not closing"
        })

        icao = 0xB7B7B7

        def inject_diverging_plane():
            here = self.get_location()
            # 230 m east: inside the 250 m detection band but beyond the 200 m well-clear radius
            contact = self.offset_location_ne(here, 0, 230)
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(contact.lat * 1e7),
                int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000),           # matched altitude -> inside the vertical gate
                9000,                            # heading 90 deg = due east (directly away)
                4000,                            # 40 m/s horizontal velocity -> opening the range
                0,
                "DIVERGE1".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,
                1, 65535, 1200,
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # inject the diverging plane steadily for 25 s; the CPA gate must keep us in AUTO
        tstart = self.get_sim_time()
        guided = 0
        samples = 0
        while self.get_sim_time() - tstart < 25:
            inject_diverging_plane()
            self.wait_heartbeat()
            samples += 1
            if self.mav.flightmode == "GUIDED":
                guided += 1
        if guided > 0:
            raise NotAchievedException(
                "loiter engaged (GUIDED in %u/%u samples) for a diverging aircraft in the outer "
                "well-clear band - conservative CPA gate should have suppressed it" % (guided, samples))
        self.disarm_vehicle(force=True)
