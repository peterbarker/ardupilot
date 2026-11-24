'''
Plane autotest: PlaneDAAAircraftLoiterNoFlip

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAAircraftLoiterNoFlip(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAAircraftLoiterNoFlip(self):
        '''planedaa must hold GUIDED steadily while a crewed aircraft contact stays
        inside the well-clear volume - it must NOT oscillate AUTO<->GUIDED (the
        log_102 loiter-latch limit cycle, caused by the stop distance being the
        bare DAA_MARGIN_CA instead of the full detection distance).  Also exercises
        the DAA_MARGIN_CA_Z vertical margin: a contact above the bare AVD_WCLR_Z but
        within AVD_WCLR_Z + DAA_MARGIN_CA_Z engages, while one above the full
        vertical gate does not.  A steady crewed aircraft (emitter LIGHT, so it takes the
        loiter-to-altitude path, not the drone bendy-ruler) is injected at ~120 m -
        beyond the 50 m DAA_MARGIN_CA (so the old code would flip) but well inside the
        250 m detection distance.  The ADSB_VEHICLE is re-sent continuously because
        AP_Avoidance prunes obstacles after 5 s.'''
        self.install_planedaa_scripts()

        # core AVD_ params exist without the script; pin the well-clear volume so the
        # geometry is deterministic: horizontal detect = AVD_WCLR_XY + DAA_MARGIN_CA
        # (200 + 50 = 250 m), vertical gate = AVD_WCLR_Z + DAA_MARGIN_CA_Z (50 + 30 = 80 m)
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,    # required for AP_Avoidance to pull ADSB samples
            "AVD_WCLR_XY": 200,
            "AVD_WCLR_Z": 50,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        # DAA_ scripting params only exist once planedaa.lua has added its table
        # (done at script load, before the STARTUP_DELAY), so set them post-reboot
        self.set_parameters({
            "DAA_MARGIN_CA": 50,
            "DAA_MARGIN_CA_Z": 30,
            "DAA_AVD_ALT": 50,
        })

        icao = 0xA5A5A5

        def inject_aircraft(dalt_m):
            # place the contact 120 m east of wherever the vehicle is now, so as it
            # loiters the horizontal separation stays ~120 m (inside 250 m, beyond 50 m)
            here = self.get_location()
            contact = self.offset_location_ne(here, 0, 120)
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(contact.lat * 1e7),
                int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000 + dalt_m * 1000),
                0,      # heading cdeg
                0,      # horizontal velocity cm/s (stationary)
                0,      # vertical velocity cm/s
                "GAJET01".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,   # is_adsb_aircraft -> loiter path
                1,      # time since last communication
                65535,  # flags
                1200,   # squawk
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # --- engage: contact at +60 m, above the bare AVD_WCLR_Z (50) but inside
        #     the AVD_WCLR_Z + DAA_MARGIN_CA_Z gate (80). Engaging proves the
        #     vertical margin is applied (a bare-WCLR_Z gate would reject +60 m).
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 60:
            inject_aircraft(60)
            if self.mav.flightmode == "GUIDED":
                break
            self.wait_heartbeat()
        if self.mav.flightmode != "GUIDED":
            raise NotAchievedException(
                "did not engage loiter (GUIDED) for a crewed aircraft inside the vertical margin band")

        # --- no-flip: hold the contact steady for 25 s; the mode must stay GUIDED
        #     the whole time. Pre-fix, the latch oscillated ~2 Hz, so any sample in
        #     AUTO here is the regression.
        tstart = self.get_sim_time()
        non_guided = 0
        samples = 0
        while self.get_sim_time() - tstart < 25:
            inject_aircraft(60)
            self.wait_heartbeat()
            samples += 1
            if self.mav.flightmode != "GUIDED":
                non_guided += 1
        if non_guided > 0:
            raise NotAchievedException(
                "aircraft loiter left GUIDED in %u/%u samples while the contact was steady "
                "(AUTO<->GUIDED latch limit cycle)" % (non_guided, samples))

        # --- release: stop injecting; the contact prunes (5 s) and the 10 s dwell
        #     expires, so the vehicle must cleanly return to AUTO.
        self.wait_mode("AUTO", timeout=30)

        # --- vertical gate upper bound: a contact above AVD_WCLR_Z + DAA_MARGIN_CA_Z
        #     (110 m > 80 m) must NOT engage the loiter, so the margin stays bounded.
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 12:
            inject_aircraft(110)
            self.wait_heartbeat()
            if self.mav.flightmode == "GUIDED":
                raise NotAchievedException(
                    "engaged loiter for a contact above the vertical gate (DAA_MARGIN_CA_Z not bounded)")
        self.disarm_vehicle(force=True)
