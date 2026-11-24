'''
Rover autotest: AISMultipleVessels

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AISMultipleVessels(TestGroup):
    @testcase(bucket="Rover")
    def AISMultipleVessels(self):
        '''Test tracking multiple AIS vessels'''
        self.customise_SITL_commandline([
            "--serial5=sim:AIS",
        ])
        self.set_parameters({
            "SERIAL5_PROTOCOL": 40,
            "SERIAL5_BAUD": 38,
            "AIS_TYPE": 1,
        })
        self.reboot_sitl()

        # Test with 1 vessel
        self.set_parameter("SIM_AIS_COUNT", 1)
        m = self.assert_receive_message('AIS_VESSEL', timeout=30)
        if m.MMSI == 0:
            raise NotAchievedException("No vessel with count=1")
        self.progress("Received vessel with count=1")

        # Test with 3 vessels
        self.set_parameter("SIM_AIS_COUNT", 3)
        vessels_seen = set()
        start_time = self.get_sim_time()
        while len(vessels_seen) < 3:
            if self.get_sim_time() - start_time > 60:
                raise NotAchievedException(
                    "Only saw %d vessels, expected 3" % len(vessels_seen)
                )
            m = self.assert_receive_message('AIS_VESSEL', timeout=10)
            vessels_seen.add(m.MMSI)
        self.progress("Tracked 3 unique vessels")

        # Test with 5 vessels
        self.set_parameter("SIM_AIS_COUNT", 5)
        vessels_seen = set()
        start_time = self.get_sim_time()
        while len(vessels_seen) < 5:
            if self.get_sim_time() - start_time > 90:
                raise NotAchievedException(
                    "Only saw %d vessels, expected 5" % len(vessels_seen)
                )
            m = self.assert_receive_message('AIS_VESSEL', timeout=10)
            vessels_seen.add(m.MMSI)

        self.progress("Successfully tracked vessels with counts 1, 3, and 5")
