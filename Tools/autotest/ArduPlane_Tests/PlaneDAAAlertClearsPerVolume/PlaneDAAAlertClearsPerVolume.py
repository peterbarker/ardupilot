'''
Plane autotest: PlaneDAAAlertClearsPerVolume

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAAlertClearsPerVolume(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAAlertClearsPerVolume(self):
        '''The NMAC and Loss of Well Clear alert states must each clear as the contact
        leaves THAT volume, not only when the contact disappears from the feed.

        alert_aircraft() used to clear both states in one place - the "aircraft_avoiding
        is nil" branch - so a contact that receded from a near miss into the well-clear
        band left NMAC latched, and the pilot was then told "Loss of Well Clear" on an
        improving situation with no "Near Miss CLEAR" in between.  A contact that receded
        further, outside well clear but still inside the detection volume, left BOTH
        latched indefinitely.

        A crewed aircraft (emitter LIGHT) is injected abeam of the vehicle and walked out
        through the two nested boundaries: 50 m (inside AVD_NMAC_XY), then 150 m (outside
        NMAC, inside AVD_WCLR_XY), then 400 m (outside well clear, still inside the
        AVD_WCLR_XY + DAA_MARGIN_CA detection volume, so it is still a live contact).
        Each step must produce its own clear message.  On the unfixed applet neither
        clear is ever sent.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "AVD_WCLR_XY": 200,
            "AVD_WCLR_Z": 50,
            "AVD_NMAC_XY": 100,
            "AVD_NMAC_Z": 30,
            "FENCE_ENABLE": 0,  # this test is about the aircraft alerts alone
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.set_parameters({
            # detection reaches AVD_WCLR_XY + DAA_MARGIN_CA = 600 m, so the contact is
            # still tracked at the final 400 m step - which is the whole point: the old
            # code only cleared when the contact fell out of the feed entirely
            "DAA_MARGIN_CA": 400,
            "DAA_MARGIN_CA_Z": 30,
            "DAA_AVD_ALT": 0,   # no loiter-to-altitude: this test is about the alerts
        })

        icao = 0xA5A5AA
        label = "%06X" % icao

        # the contact is re-sent off the position stream rather than from a poll loop, and
        # sits abeam of the vehicle at whatever separation the current phase asks for, so
        # the geometry is exact no matter how the vehicle manoeuvres
        contact_offset_m = [50]

        def inject_contact(mav, m):
            if m.get_type() != 'GLOBAL_POSITION_INT':
                return
            here = Location.latlon_only(m.lat * 1e-7, m.lon * 1e-7)
            contact = self.offset_location_ne(here, 0, contact_offset_m[0])
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(contact.lat * 1e7),
                int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                m.alt,                                      # co-altitude, mm AMSL
                0, 0, 0,
                "GAJET07".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,    # crewed
                1, 65535, 1200,
            )

        def require_text(text, why, regex=False):
            try:
                self.wait_text(text, check_context=True, timeout=90, regex=regex)
            except AutoTestTimeoutException:
                raise NotAchievedException(why)

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2600, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)
        self.install_message_hook_context(inject_contact)

        # --- 50 m abeam: inside AVD_NMAC_XY, a near miss
        require_text("Near Miss", "a contact 50 m abeam inside AVD_NMAC_XY raised no near-miss alert")
        self.progress("near miss raised - backing the contact out of AVD_NMAC_XY")

        # --- 150 m abeam: outside NMAC, still inside AVD_WCLR_XY
        contact_offset_m[0] = 150
        require_text(
            "Near Miss CLEAR",
            "the contact left AVD_NMAC_XY (150 m abeam, NMAC is 100 m) but the near miss "
            "was never cleared: NMAC is still latched until the contact vanishes")
        require_text(
            "Loss of Well Clear",
            "the contact inside AVD_WCLR_XY raised no loss-of-well-clear alert")
        self.progress("near miss cleared while still inside well clear - backing further out")

        # --- 400 m abeam: outside well clear, still a live contact (detection is 600 m)
        contact_offset_m[0] = 400
        require_text(
            ".*%s Well Clear" % label,   # re.match is anchored; must not match "Loss of Well Clear"
            "the contact left AVD_WCLR_XY (400 m abeam, well clear is 200 m) but well "
            "clear was never restored: LoWC is still latched until the contact vanishes",
            regex=True)
        self.progress("well clear restored with the contact still being tracked")
        self.disarm_vehicle(force=True)
