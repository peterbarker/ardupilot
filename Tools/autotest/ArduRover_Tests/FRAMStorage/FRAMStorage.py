'''
Rover autotest: FRAMStorage

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class FRAMStorage(TestGroup):
    @testcase(bucket="Rover")
    def FRAMStorage(self):
        '''Test FRAM storage (for parameters etc)'''
        self.set_parameter("LOG_BITMASK", 1)
        self.reboot_sitl()

        self.customise_SITL_commandline([
            "--set-storage-posix-enabled", "0",
            "--set-storage-fram-enabled", "1",
        ])
        # TODO: ensure w'ere actually taking stuff from flash storage:
#        if self.get_parameter("LOG_BITMASK") == 1:
#            raise NotAchievedException("not using flash storage?")
        self.set_parameter("LOG_BITMASK", 2)
        self.reboot_sitl()
        self.assert_parameter_value("LOG_BITMASK", 2)
        self.set_parameter("LOG_BITMASK", 3)
        self.reboot_sitl()
        self.assert_parameter_value("LOG_BITMASK", 3)

        self.customise_SITL_commandline([])
        # make sure we're back at our original value:
        self.assert_parameter_value("LOG_BITMASK", 1)
