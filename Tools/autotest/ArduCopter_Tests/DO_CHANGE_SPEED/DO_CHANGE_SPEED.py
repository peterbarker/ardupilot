'''
Copter autotest: DO_CHANGE_SPEED

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class DO_CHANGE_SPEED(TestGroup):
    @testcase(bucket="CopterTests1d")
    def DO_CHANGE_SPEED(self):
        '''Change speed during mission using waypoint items'''
        self.load_mission("mission.txt", strict=False)

        self.set_parameters({
            "AUTO_OPTIONS": 3,
            "ATC_ANGLE_MAX": 45,
        })

        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.wait_current_waypoint(1)
        self.wait_groundspeed(
            3.5, 4.5,
            minimum_duration=5,
            timeout=60,
        )

        self.wait_current_waypoint(3)
        self.wait_groundspeed(
            14.5, 15.5,
            minimum_duration=10,
            timeout=60,
        )

        self.wait_current_waypoint(5)
        self.wait_groundspeed(
            9.5, 11.5,
            minimum_duration=10,
            timeout=60,
        )

        self.set_parameter("ATC_ANGLE_MAX", 60)
        self.wait_current_waypoint(7)
        self.wait_groundspeed(
            15.5, 16.5,
            minimum_duration=10,
            timeout=60,
        )

        self.wait_disarmed()
