'''
Plane autotest: SetHomeAltChange

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class SetHomeAltChange(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def SetHomeAltChange(self):
        '''check modes retain altitude when home alt changed'''
        for mode in 'FBWB', 'CRUISE', 'LOITER':
            self.set_rc(3, 1000)
            self.wait_ready_to_arm()
            home = self.home_position_as_location()
            home_alt_amsl = home.get_alt_m(AltFrame.ABSOLUTE)
            target_alt = 20
            self.takeoff(target_alt, mode="TAKEOFF")
            self.wait_altitude(
                home_alt_amsl+target_alt-5,
                home_alt_amsl+target_alt+5,
                relative=False,
                minimum_duration=20,
                timeout=60,
            )
            self.set_rc(3, 1500)
            self.change_mode(mode)
            self.set_home(self.offset_location_up(home, 40))
            self.wait_altitude(
                home_alt_amsl+target_alt-5,
                home_alt_amsl+target_alt+5,
                relative=False,
                minimum_duration=10,
                timeout=15,
            )
            self.disarm_vehicle(force=True)
            self.reboot_sitl()
