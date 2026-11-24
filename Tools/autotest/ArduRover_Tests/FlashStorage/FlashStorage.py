'''
Rover autotest: FlashStorage

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FlashStorage(TestGroup):
    @testcase(bucket="Rover")
    def FlashStorage(self):
        '''Test flash storage (for parameters etc)'''
        self.set_parameter("LOG_BITMASK", 1)
        self.reboot_sitl()

        self.customise_SITL_commandline([
            "--set-storage-posix-enabled", "0",
            "--set-storage-flash-enabled", "1",
        ])
        if self.get_parameter("LOG_BITMASK") == 1:
            raise NotAchievedException("not using flash storage?")
        self.set_parameter("LOG_BITMASK", 2)
        self.reboot_sitl()
        self.assert_parameter_value("LOG_BITMASK", 2)
        self.set_parameter("LOG_BITMASK", 3)
        self.reboot_sitl()
        self.assert_parameter_value("LOG_BITMASK", 3)

        self.customise_SITL_commandline([])
        # make sure we're back at our original value:
        self.assert_parameter_value("LOG_BITMASK", 1)
