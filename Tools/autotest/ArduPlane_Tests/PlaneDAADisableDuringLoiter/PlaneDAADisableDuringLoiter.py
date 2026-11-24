'''
Plane autotest: PlaneDAADisableDuringLoiter

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAADisableDuringLoiter(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAADisableDuringLoiter(self):
        '''Switching DAA off during an aircraft avoidance loiter must hand the vehicle
        back to the mode it was flying before.

        The crewed-aircraft response is a loiter-to-altitude, which hijacks the flight
        MODE - loiteralt.start() commands GUIDED and the loiter point directly rather
        than going through set_avoid_location() - so daa_target_loc stays nil.
        DAA.clear_avoidance() used to return immediately on that nil, and nothing else
        could stop the loiter either, because loiteralt.update() is only reached from
        do_loitering(), which runs only while DAA.isactive().  Switching DAA off
        mid-loiter therefore left the plane circling in GUIDED indefinitely.

        This is the same defect PlaneDAADisableRevertsTarget covers for the
        target-hijack path; only the loiter path was left uncovered.

        The contact keeps being injected after the switch is thrown, so a release
        caused by the contact pruning cannot pass this test by accident.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,    # required for AP_Avoidance to pull ADSB samples
            "AVD_WCLR_XY": 200,
            "AVD_WCLR_Z": 50,
            "RC7_OPTION": 308,  # DAA on/off aux switch (DAA_ACT_FN); low = on, high = off
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        # DAA_ scripting params only exist once planedaa.lua has added its table
        self.set_parameters({
            "DAA_MARGIN_CA": 50,
            "DAA_MARGIN_CA_Z": 30,
            "DAA_AVD_ALT": 50,
        })

        # drive the aux switch LOW = DAA on.  An unset channel does not read as low, and
        # the script disables DAA for any switch position at or above the middle, so
        # without this DAA is off from boot and the loiter never engages.
        self.set_rc(7, 1000)

        icao = 0xA5A5A6

        def inject_aircraft():
            # 120 m abeam: inside the 250 m detection distance, beyond DAA_MARGIN_CA
            here = self.get_location()
            contact = self.offset_location_ne(here, 0, 120)
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(contact.lat * 1e7),
                int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 60 * 1000),
                0,      # heading cdeg
                0,      # horizontal velocity cm/s (stationary)
                0,      # vertical velocity cm/s
                "GAJET02".encode("ascii"),
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

        # --- engage the loiter: the vehicle must be in GUIDED, having come from AUTO
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 60:
            inject_aircraft()
            if self.mav.flightmode == "GUIDED":
                break
            self.wait_heartbeat()
        if self.mav.flightmode != "GUIDED":
            raise NotAchievedException("did not engage the aircraft loiter (GUIDED)")

        # --- switch DAA off while the loiter is running, and keep the contact alive
        self.set_rc(7, 2000)

        tstart = self.get_sim_time()
        released = False
        while self.get_sim_time() - tstart < 30:
            inject_aircraft()          # deliberately keep the threat present
            self.wait_heartbeat()
            if self.mav.flightmode == "AUTO":
                released = True
                break
        if not released:
            raise NotAchievedException(
                "DAA was switched off during an aircraft loiter but the vehicle stayed "
                "in %s: the loiter was never stopped and the saved mode never restored"
                % self.mav.flightmode)

        # --- and it must stay handed back, not re-engage with DAA off
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 15:
            inject_aircraft()
            self.wait_heartbeat()
            if self.mav.flightmode != "AUTO":
                raise NotAchievedException(
                    "re-entered %s with DAA switched off" % self.mav.flightmode)
        self.disarm_vehicle(force=True)
