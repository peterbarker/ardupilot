'''
Plane autotest: SetHomeAltChange3

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class SetHomeAltChange3(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def SetHomeAltChange3(self):
        '''same as SetHomeAltChange, but the home alt change occurs during TECS operation'''
        self.wait_ready_to_arm()
        home = self.home_position_as_location()
        home_alt_amsl = home.get_alt_m(AltFrame.ABSOLUTE)
        target_alt = 20
        self.takeoff(target_alt, mode="TAKEOFF")
        self.change_mode("LOITER")
        self.delay_sim_time(20, reason="plane to settle") # Let the plane settle.

        self.set_home(self.offset_location_up(home, 40))
        self.wait_altitude(
            home_alt_amsl+target_alt-5,
            home_alt_amsl+target_alt+5,
            relative=False,
            minimum_duration=10,
            timeout=10.1,
        )

        self.disarm_vehicle(force=True)
        self.reboot_sitl()
