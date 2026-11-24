'''
QuadPlane autotest: Weathervane

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class Weathervane(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def Weathervane(self):
        '''test nose-into-wind functionality'''
        # We test nose into wind code paths and yaw direction in copter autotest,
        # so we shall test the side into wind yaw direction and plane code paths here.
        self.set_parameters({"SIM_WIND_SPD": 10,
                             "SIM_WIND_DIR": 240,
                             "Q_WVANE_ENABLE": 3, # WVANE_ENABLE = 3 gives direction of side into wind
                             "Q_WVANE_GAIN": 3,
                             "STICK_MIXING": 0})

        self.takeoff(10, mode="QLOITER")

        # Turn aircraft to heading 90 deg
        self.set_rc(4, 1700)
        self.wait_heading(90)
        self.set_rc(4, 1500)

        # Now wait for weathervaning to activate and turn side-on to wind at 240 deg therefore heading 150 deg
        self.wait_heading(150, accuracy=5, timeout=180)

        self.do_RTL()
