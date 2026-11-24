'''
Plane autotest: DeepStall

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class DeepStall(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def DeepStall(self):
        '''Test DeepStall Landing'''
        # self.fly_deepstall_absolute()
        self.fly_deepstall_relative()

    def fly_deepstall_relative(self):
        self.start_subtest("DeepStall Relative")
        deepstall_elevator_pwm = 1661
        self.set_parameters({
            "LAND_TYPE": 1,
            "LAND_DS_ELEV_PWM": deepstall_elevator_pwm,
            "RTL_AUTOLAND": 1,
        })
        self.start_flying_mission("plane-deepstall-relative-mission.txt")
        self.wait_current_waypoint(4)

        # assume elevator is on channel 2:
        self.wait_servo_channel_value(2, deepstall_elevator_pwm, timeout=240)

        self.progress("Waiting for stage DEEPSTALL_STAGE_LAND")
        self.assert_receive_message(
            'DEEPSTALL',
            condition='DEEPSTALL.stage==6',
            timeout=240,
        )
        self.progress("Reached stage DEEPSTALL_STAGE_LAND")

        self.disarm_wait(timeout=120)
        self.set_current_waypoint(0, check_afterwards=False)

        self.progress("Flying home")
        self.set_current_waypoint(0, check_afterwards=False)
        self.takeoff(100)
        self.set_parameter("LAND_TYPE", 0)
        self.fly_home_land_and_disarm(timeout=240)
