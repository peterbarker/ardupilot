'''
Copter autotest: TerrainDBPreArm

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TerrainDBPreArm(TestGroup):
    @testcase(bucket="CopterTests2b")
    def TerrainDBPreArm(self):
        '''test that pre-arm checks are working correctly for terrain database'''
        self.context_push()

        self.progress("# Load msission with terrain alt")
        # load the waypoint
        num_wp = self.load_mission("terrain_wp.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load terrain_wp failed")

        self.set_analog_rangefinder_parameters()
        self.set_parameters({
            "WP_RFND_USE": 1,
            "TERRAIN_ENABLE": 1,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()

        # make sure we can still arm with valid rangefinder and terrain db disabled
        self.set_parameter("TERRAIN_ENABLE", 0)
        self.wait_ready_to_arm()
        self.progress("# Vehicle armed with terrain db disabled")

        # make sure we can't arm with terrain db enabled and no rangefinder in us
        self.set_parameter("WP_RFND_USE", 0)
        self.assert_prearm_failure("terrain disabled")

        self.context_pop()

        self.reboot_sitl()
