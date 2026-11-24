'''
Copter autotest: LuaCopterCircleSpeed

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class LuaCopterCircleSpeed(TestGroup):
    # test copter-circle-speed lua script:
    @testcase(bucket="CopterTests1d")
    def LuaCopterCircleSpeed(self):
        '''test the consistent-ground-speed-circling works'''
        self.install_example_script_context('copter-circle-speed.lua')
        self.set_parameters({
            "SCR_ENABLE": 1,
            "WP_ACC": 5,
        })
        self.reboot_sitl()

        expected_groundspeed = 3
        self.set_parameter("CSPD_SPEED", expected_groundspeed)
        radius_m = self.get_parameter('CIRCLE_RADIUS_M')
        circle_point = self.offset_location_heading_distance(
            self.get_location(),
            270,  # see SITL_START_LOCATION, FIXME!
            radius_m
        )

        self.change_mode('LOITER')
        self.wait_ready_to_arm()

        self.takeoff(10, mode='LOITER')
        self.hover()  # circle mode uses throttle input
        self.change_mode('CIRCLE')

        self.wait_groundspeed(
            expected_groundspeed-0.2,
            expected_groundspeed+0.2,
            minimum_duration=10,
        )
        self.set_rc(2, 1550)  # pilot increases radius
        self.wait_distance_to_location(circle_point, radius_m*2-1, radius_m*2+1)
        self.set_rc(2, 1500)
        self.wait_groundspeed(
            expected_groundspeed-0.2,
            expected_groundspeed+0.2,
            minimum_duration=10,
        )
        self.do_RTL()
