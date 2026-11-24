'''
Rover autotest: AIS

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AIS(TestGroup):
    @testcase(bucket="Rover")
    def AIS(self):
        '''Test AIS receiver'''
        self.customise_SITL_commandline([
            "--serial5=sim:AIS",
        ])
        self.set_parameters({
            "SERIAL5_PROTOCOL": 40,
            "SERIAL5_BAUD": 38,
            "AIS_TYPE": 1,
        })
        self.reboot_sitl()

        # Set SIM parameter after reboot when AIS sim is loaded
        self.set_parameter("SIM_AIS_COUNT", 5)

        m = self.assert_receive_message('AIS_VESSEL', timeout=60)

        if m.MMSI == 0:
            raise NotAchievedException("Invalid MMSI")

        self.progress("Received AIS vessel MMSI=%u" % m.MMSI)
