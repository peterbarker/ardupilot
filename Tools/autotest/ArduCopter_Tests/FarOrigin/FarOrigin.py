'''
Copter autotest: FarOrigin

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FarOrigin(TestGroup):
    @testcase(bucket="CopterTests2b")
    def FarOrigin(self):
        '''fly a mission far from the vehicle origin'''
        # Fly mission #1
        self.set_parameters({
            "SIM_GPS1_ENABLE": 0,
        })
        self.reboot_sitl()
        nz = Location(-43.730171, 169.983118, 1466.3, AltFrame.ABSOLUTE)
        self.set_origin(nz)
        self.set_parameters({
            "SIM_GPS1_ENABLE": 1,
        })
        self.progress("# Load copter_mission")
        # load the waypoint count
        num_wp = self.load_mission("copter_mission.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load copter_mission failed")

        self.fly_loaded_mission(num_wp)

        self.progress("Auto mission completed: passed!")
