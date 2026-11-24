'''
Plane autotest: ADSBFailActionRTL

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ADSBFailActionRTL(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def ADSBFailActionRTL(self):
        '''test ADSB avoidance action of RTL'''
        # message ADSB_VEHICLE 37 -353632614 1491652305 0 584070 0 0 0 "bob" 3 1 255 17
        self.set_parameter("RC12_OPTION", 38) # avoid-adsb
        self.set_rc(12, 2000)
        self.set_parameters({
            "ADSB_TYPE": 1,
            "AVD_ENABLE": 1,
            "AVD_F_ACTION": mavutil.mavlink.MAV_COLLISION_ACTION_RTL,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        here = self.get_location()
        self.change_mode("FBWA")
        self.delay_sim_time(2, reason="mode change to settle") # TODO: work out why this is required...
        self.test_adsb_send_threatening_adsb_message(here)
        self.progress("Waiting for collision message")
        m = self.assert_receive_message('COLLISION', timeout=4)
        if m.threat_level != 2:
            raise NotAchievedException("Expected some threat at least")
        if m.action != mavutil.mavlink.MAV_COLLISION_ACTION_RTL:
            raise NotAchievedException("Incorrect action; want=%u got=%u" %
                                       (mavutil.mavlink.MAV_COLLISION_ACTION_RTL, m.action))
        self.wait_mode("RTL")

        self.progress("Sending far-away ABSD_VEHICLE message")
        self.mav.mav.adsb_vehicle_send(
            37, # ICAO address
            int(here.lat+1 * 1e7),
            int(here.lng * 1e7),
            mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
            int(here.get_alt_m(AltFrame.ABSOLUTE)*1000 + 10000), # 10m up
            0, # heading in cdeg
            0, # horizontal velocity cm/s
            0, # vertical velocity cm/s
            "bob".encode("ascii"), # callsign
            mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,
            1, # time since last communication
            65535, # flags
            17 # squawk
        )
        self.wait_for_collision_threat_to_clear()
        self.change_mode("FBWA")

        self.progress("Disabling ADSB-avoidance with RC channel")
        self.set_rc(12, 1000)
        self.delay_sim_time(1, reason="RC switch to be polled") # let the switch get polled
        self.test_adsb_send_threatening_adsb_message(here)
        m = self.assert_not_receive_message('COLLISION', timeout=4)

        # AP_Avoidance::disable() writes AVD_ENABLE's live value.  Not
        # saving it is correct - a switch position must not change what
        # is stored - but writing the parameter at all is a firmware bug:
        # the switch should drive a separate runtime flag, as
        # AC_Avoid::proximity_avoidance_enable() does.  Until that is
        # fixed, leaving the switch down means the live value reads 0
        # while storage holds the 1 we set above, so the context restore
        # asks for 0, sees 0, writes nothing, and the next boot in this
        # session comes up with avoidance enabled.  Put the switch back.
        self.set_rc(12, 2000)
        self.delay_sim_time(1, reason="RC switch to be polled")
