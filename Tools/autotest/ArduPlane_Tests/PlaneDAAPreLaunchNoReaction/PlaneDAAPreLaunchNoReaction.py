'''
Plane autotest: PlaneDAAPreLaunchNoReaction

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAPreLaunchNoReaction(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAPreLaunchNoReaction(self):
        '''in_takeoff_or_landing's is_taking_off() only reads true once Plane's flight
        stage reaches TAKEOFF, which does not happen until the launch itself triggers
        (throttle no longer suppressed) - so a vehicle armed in AUTO on a NAV_TAKEOFF
        item, still waiting for that trigger, is NOT covered by is_taking_off() alone.
        Without an additional check, a contact close enough to demand avoidance during
        that wait can command a target/mode change before the vehicle has ever left
        the ground, which can itself prevent the launch from ever triggering.

        TKOFF_THR_MINACC holds the vehicle in that pre-launch wait indefinitely (the
        same mechanism a catapult/bungee/hand launch relies on) since SITL sitting
        still on the ground never produces the forward acceleration event it is
        waiting for. A crewed-aircraft contact close enough to demand avoidance is
        injected throughout the wait; the vehicle must neither react to it nor leave
        AUTO. Releasing the hold afterwards confirms the wait was real and the launch
        still proceeds normally once airborne.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "AVD_WCLR_XY": 500,
            "AVD_WCLR_Z": 50,
            "TKOFF_THR_MINACC": 15,    # hold on the ground until accel triggers the launch
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        home = self.home_position_as_location()
        # close enough to demand avoidance, well inside AVD_WCLR_XY/Z
        contact_loc = self.offset_location_ne(home, 425, 0)
        icao = 0xF000A2

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
                "PRELNCH1".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_SMALL,   # emitter 2 -> CREWED_AIRCRAFT
                1, 65535, 1200,
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # Detection/alerting are expected to keep running in this stage (the contact
        # really is within AVD_WCLR_XY/Z, so "Loss of Well Clear"/"Near Miss" alerts
        # are legitimate) - only a commanded reaction (a target or mode change) is
        # forbidden here.
        self.daa_watch_no_reaction(20, [
            "AVOIDING", "LOITERING", "LOITER AIRCRAFT", "TRAPPED",
        ], inject)

        if self.mav.flightmode != "AUTO":
            raise NotAchievedException(
                "planedaa commanded a mode change (now %s) before launch" % self.mav.flightmode)
        if self.get_altitude(relative=True) > 2:
            raise NotAchievedException(
                "vehicle left the ground during the pre-launch wait - the wait was not real")

        # release the hold and confirm the launch still happens normally
        self.set_parameter("TKOFF_THR_MINACC", 0)
        self.wait_text("Takeoff complete", check_context=True, timeout=30)
        self.wait_current_waypoint(2, timeout=120)
        self.disarm_vehicle(force=True)
