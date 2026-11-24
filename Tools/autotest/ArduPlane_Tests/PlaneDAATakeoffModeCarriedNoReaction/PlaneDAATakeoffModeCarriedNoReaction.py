'''
Plane autotest: PlaneDAATakeoffModeCarriedNoReaction

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAATakeoffModeCarriedNoReaction(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAATakeoffModeCarriedNoReaction(self):
        '''PlaneDAAPreLaunchNoReaction only covers AUTO: vehicle:get_likely_flying()'s
        pre-launch override (update_is_flying_5Hz(), ArduPlane/is_flying.cpp) is itself
        gated to control_mode == mode_auto, so it does not apply in the dedicated
        Takeoff flight mode. There, GPS ground speed above the 1.5 m/s is_flying()
        threshold can already read as flying before the launch trigger fires - e.g. a
        carried or bungee launch, held just before release. SIM_GPS1_VERR_X stands in
        for that ground speed. A height-above-home backstop withholds in this mode
        specifically; this test holds the pre-launch wait with TKOFF_THR_MINACC, both
        stationary and then under simulated GPS speed noise, and checks for no reaction
        throughout, then releases the hold and confirms a normal climb-out.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "AVD_WCLR_XY": 500,
            "AVD_WCLR_Z": 50,
            "TKOFF_THR_MINACC": 15,    # hold on the ground until accel triggers the launch
            "TKOFF_ALT": 50,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.change_mode("TAKEOFF")
        self.arm_vehicle()

        home = self.home_position_as_location()
        # close enough to demand avoidance, well inside AVD_WCLR_XY/Z
        contact_loc = self.offset_location_ne(home, 425, 0)
        icao = 0xF000A3

        sample_count = [0]

        def inject():
            sample_count[0] += 1
            # see PlaneDAAParkedAircraftIgnored's inject() for why this jitters
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(contact_loc.lat * 1e7) + (sample_count[0] % 2),
                int(contact_loc.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(home.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 40000),  # 40 m up
                0, 0, 0,       # heading, hor/vert velocity: stationary
                "PRELNCH2".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_SMALL,   # emitter 2 -> CREWED_AIRCRAFT
                1, 65535, 1200,
            )

        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # Detection/alerting are expected to keep running in this stage (see
        # PlaneDAAPreLaunchNoReaction) - only a commanded reaction is forbidden here.
        forbidden_texts = ["AVOIDING", "LOITERING", "LOITER AIRCRAFT", "TRAPPED"]

        # stationary first: confirm the wait holds with no GPS noise at all
        self.daa_watch_no_reaction(5, forbidden_texts, inject)

        # now simulate being carried: GPS ground speed noise above the is_flying()
        # threshold, with the launch trigger (TKOFF_THR_MINACC) still withheld
        self.set_parameter("SIM_GPS1_VERR_X", 3)
        self.daa_watch_no_reaction(15, forbidden_texts, inject)
        self.set_parameter("SIM_GPS1_VERR_X", 0)

        if self.mav.flightmode != "TAKEOFF":
            raise NotAchievedException(
                "planedaa commanded a mode change (now %s) before launch" % self.mav.flightmode)
        if self.get_altitude(relative=True) > 2:
            raise NotAchievedException(
                "vehicle left the ground during the pre-launch wait - the wait was not real")

        # release the hold and confirm the launch still happens normally
        self.set_parameter("TKOFF_THR_MINACC", 0)
        self.wait_altitude(48, 52, relative=True, timeout=60)
        self.disarm_vehicle(force=True)
