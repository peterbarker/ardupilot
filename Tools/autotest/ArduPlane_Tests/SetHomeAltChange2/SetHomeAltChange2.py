'''
Plane autotest: SetHomeAltChange2

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class SetHomeAltChange2(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def SetHomeAltChange2(self):
        '''ensure TECS operates predictably as home altitude changes continuously'''
        '''
        This can happen when performing a ship landing, where the home
        coordinates are continuously set by the ship GNSS RX.
        '''
        self.set_parameter('TRIM_THROTTLE', 70)
        self.wait_ready_to_arm()
        home = self.home_position_as_location()
        home_alt_amsl = home.get_alt_m(AltFrame.ABSOLUTE)
        target_alt = 20
        self.takeoff(target_alt, mode="TAKEOFF")
        self.change_mode("LOITER")
        self.delay_sim_time(20, reason="plane to settle") # Let the plane settle.

        tstart = self.get_sim_time()
        test_time = 10 # Run the test for 10s.
        pub_freq = 10
        for i in range(test_time*pub_freq):
            tnow = self.get_sim_time()
            # Produce 1Hz sine waves in home altitude change.
            self.set_home(self.offset_location_up(home, 40*math.sin((tnow-tstart)*(2*math.pi))))
            if tnow-tstart > test_time:
                break
            self.delay_sim_time(1.0/pub_freq, reason="home update interval")

        # Test if the altitude is still within bounds.
        self.wait_altitude(
            home_alt_amsl+target_alt-5,
            home_alt_amsl+target_alt+5,
            relative=False,
            minimum_duration=1,
            timeout=2,
        )
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
