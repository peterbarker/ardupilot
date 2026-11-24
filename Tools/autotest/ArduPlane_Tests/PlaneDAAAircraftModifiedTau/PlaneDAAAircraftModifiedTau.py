'''
Plane autotest: PlaneDAAAircraftModifiedTau

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAAircraftModifiedTau(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAAircraftModifiedTau(self):
        '''A physically closing contact must trigger the measured-range tau branch.

        Warm up with the loiter disabled so the first-fix velocity fallback cannot
        pass the test. Then enable the loiter and require it before entering the
        standoff. Unlike the existing converging test, the contact follows a fixed
        world trajectory rather than following the vehicle at a constant range.
        '''
        self.install_planedaa_scripts()
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,
            "AVD_ENABLE": 1,
            "AVD_WCLR_XY": 300,
            # Above the bendy-ruler vertical gate, inside the aircraft detector's
            # extended gate: horizontal avoidance cannot change the approach.
            "AVD_WCLR_Z": 20,
        })
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameters({
            "DAA_AVD_ALT": 0,
            "DAA_AVD_ALT_TP": 1,
            "DAA_MARGIN_CA": 2000,
            "DAA_MARGIN_CA_Z": 100,
            "DAA_TAU_S": 30,
            "DAA_TRAP_ACT": 0,
        })
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 100),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 5000, 0, 100),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)
        self.wait_altitude(95, 105, relative=True, timeout=120)
        self.delay_sim_time(8, reason="settle before the closing contact")
        origin = self.get_location()
        tstart = self.get_sim_time()
        enabled = False
        while self.get_sim_time() - tstart < 80:
            elapsed = self.get_sim_time() - tstart
            # A southbound aircraft at 15 m/s, initially 2 km north of us.
            contact = self.offset_location_ne(origin, 2000 - 15 * elapsed, 0)
            self.mav.mav.adsb_vehicle_send(
                0xABC124, int(contact.lat * 1e7), int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int((origin.get_alt_m(AltFrame.ABSOLUTE) + 50) * 1000),
                18000, 1500, 0, b"TAUCLOSE", mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,
                1, 65535, 1200,
            )
            if elapsed >= 8 and not enabled:
                self.set_parameter("DAA_AVD_ALT", 50)
                enabled = True
            self.wait_heartbeat()
            distance = self.get_distance(contact, self.get_location())
            if distance <= 400:
                raise NotAchievedException("Modified tau did not engage before the standoff: %.0fm" % distance)
            if self.mav.flightmode == "GUIDED":
                if not enabled:
                    raise NotAchievedException("Loiter engaged during tau warm-up")
                self.progress("Modified tau engaged at %.0fm" % distance)
                break
        else:
            raise NotAchievedException("Modified tau never engaged the aircraft loiter")
        self.disarm_vehicle(force=True)
