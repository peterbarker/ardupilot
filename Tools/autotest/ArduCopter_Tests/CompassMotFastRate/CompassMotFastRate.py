'''
Copter autotest: CompassMotFastRate

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class CompassMotFastRate(TestGroup):
    @testcase(bucket="CopterTests2b")
    def CompassMotFastRate(self):
        '''test compassmot with the rate thread driving the motors'''
        self.set_parameter("FSTRATE_ENABLE", 1)
        self.reboot_sitl()
        self.CompassMot()
