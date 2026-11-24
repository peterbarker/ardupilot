'''
Copter autotest: Ch6TuningLoitMaxXYSpeed

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class Ch6TuningLoitMaxXYSpeed(TestGroup):
    @testcase(bucket="CopterTests2c")
    def Ch6TuningLoitMaxXYSpeed(self):
        '''test loiter can be changed via Ch6 tuning knob'''
        self.set_parameters({
            "RC6_OPTION": 219,  # RC6 used for tuning
            "TUNE": 60,  # 60 is x/y loiter speed
            "TUNE_MIN": 0.2,  # 0.2m/s
            "TUNE_MAX": 10,  # 10m/s
            "AUTO_OPTIONS": 3,
        })
        self.set_rc(6, 2000)
        self.reboot_sitl()

        self.takeoff(mode='LOITER')

        self.set_rc(2, 1000)

        self.wait_groundspeed(9.5, 10.5, minimum_duration=5)

        self.set_rc(6, 1500)
        self.wait_groundspeed(4.5, 5.5, minimum_duration=5)

        self.set_rc(6, 2000)
        self.wait_groundspeed(9.5, 10.5, minimum_duration=5)

        self.set_rc(6, 1300)
        self.wait_groundspeed(2.5, 3.5, minimum_duration=5)

        self.set_rc(2, 1500)

        self.do_RTL()
