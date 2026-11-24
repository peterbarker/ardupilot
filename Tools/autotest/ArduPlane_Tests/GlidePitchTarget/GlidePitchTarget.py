'''
Plane autotest: GlidePitchTarget

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GlidePitchTarget(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def GlidePitchTarget(self):
        '''Ensure GLIDE holds STAB_PITCH_DOWN and ignores the flare switch'''
        self.set_parameters({
            "RC9_OPTION": 89,      # landing flare switch
            "LAND_PITCH_DEG": 15,
            "STAB_PITCH_DOWN": 8,
        })
        self.takeoff(100, mode='TAKEOFF')
        self.change_mode('FBWA')
        self.set_rc(3, 1000)  # throttle at zero thrust

        self.progress("Flare switch high holds LAND_PITCH_DEG in FBWA")
        self.set_rc(9, 2000)
        self.wait_pitch(15, accuracy=5, timeout=10)

        self.progress("GLIDE must hold -STAB_PITCH_DOWN with the flare switch still high")
        self.change_mode(27)  # Glide, need pymavlink support to use text name
        self.wait_pitch(-8, accuracy=4, timeout=10, minimum_duration=5)

        self.progress("GLIDE pitch must not go below PTCH_LIM_MIN_DEG")
        self.set_parameter("PTCH_LIM_MIN_DEG", -2)
        self.wait_pitch(-2, accuracy=2, timeout=10, minimum_duration=5)
        self.set_parameter("PTCH_LIM_MIN_DEG", -20)

        self.set_rc(9, 1000)
        self.change_mode('FBWA')
        self.set_rc(3, 1500)
        self.fly_home_land_and_disarm()
