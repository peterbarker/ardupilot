'''
Plane autotest: PlaneDAAAvdAltZeroNoLoiter

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAAvdAltZeroNoLoiter(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAAvdAltZeroNoLoiter(self):
        '''DAA_AVD_ALT = 0 must disable the crewed-aircraft loiter, as its documentation
        says ("Ignored if zero").

        It did not.  loiteralt.start() was called unconditionally, so at zero the vehicle
        still changed to GUIDED and commanded a target altitude of 0, which
        set_vehicle_target_location() then refused - leaving a mode change, a
        "set_vehicle FAILED" message and no way for an operator to turn the loiter off at
        all.  (loiteralt.start() no longer changes mode itself, so a refusal today leaves
        the mode alone; the missing-parameter-check this test guards is unaffected.)

        A steady crewed aircraft is held inside the well-clear volume, which is exactly the
        condition that triggers the loiter.  With the parameter at zero the vehicle must
        stay in AUTO and must never announce the loiter; ordinary bendy-ruler avoidance
        still applies, which is what falling through to monitoring gives us.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "AVD_WCLR_XY": 200,
            "AVD_WCLR_Z": 50,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.set_parameters({
            "DAA_MARGIN_CA": 50,
            "DAA_MARGIN_CA_Z": 30,
            "DAA_AVD_ALT": 0,      # the parameter under test: no loiter-to-altitude
        })

        icao = 0xA5A5A8

        def inject_aircraft():
            # 120 m abeam and 10 m up: comfortably inside AVD_WCLR_XY + DAA_MARGIN_CA (250 m)
            # and AVD_WCLR_Z + DAA_MARGIN_CA_Z (80 m), i.e. a certain loiter trigger
            here = self.get_location()
            contact = self.offset_location_ne(here, 0, 120)
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(contact.lat * 1e7),
                int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 10 * 1000),
                0, 0, 0,
                "GAJET04".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,   # crewed -> loiter path
                1, 65535, 1200,
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # hold the contact inside the well-clear volume; the loiter must never engage
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 60:
            inject_aircraft()
            self.wait_heartbeat()
            if self.mav.flightmode == "GUIDED":
                raise NotAchievedException(
                    "entered GUIDED with DAA_AVD_ALT = 0: the loiter-to-altitude is not "
                    "disabled, and its documentation says it is ignored at zero")

        if self.statustext_in_collections("LOITER AIRCRAFT") is not None:
            raise NotAchievedException(
                "announced the aircraft loiter with DAA_AVD_ALT = 0")
        if self.statustext_in_collections("set_vehicle FAILED") is not None:
            raise NotAchievedException(
                "loiter was attempted and failed to command altitude 0 with DAA_AVD_ALT = 0")

        # the contact was genuinely inside the volume, so the aircraft must have been seen -
        # otherwise this test would pass simply by nothing being detected
        if self.statustext_in_collections("ALERT") is None:
            raise NotAchievedException(
                "no aircraft ALERT: the contact was never detected, so the test proves nothing")
        self.disarm_vehicle(force=True)
