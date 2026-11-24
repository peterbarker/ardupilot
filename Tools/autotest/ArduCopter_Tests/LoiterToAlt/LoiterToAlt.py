'''
Copter autotest: LoiterToAlt

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class LoiterToAlt(TestGroup):
    @testcase(bucket="CopterTests2d")
    def LoiterToAlt(self):
        """Loiter-To-Alt"""

        self.context_push()

        self.set_parameters({
            "PLND_ENABLED": 1,
            "PLND_TYPE": 4,
        })

        self.set_analog_rangefinder_parameters()

        self.reboot_sitl()

        num_wp = self.load_mission("copter_loiter_to_alt.txt")

        self.change_mode('LOITER')

        self.install_terrain_handlers_context()
        self.wait_ready_to_arm()

        self.arm_vehicle()

        self.change_mode('AUTO')

        self.set_rc(3, 1550)

        self.wait_current_waypoint(2)

        self.set_rc(3, 1500)

        self.wait_waypoint(0, num_wp-1, timeout=500)

        self.wait_disarmed()

        self.context_pop()
        self.reboot_sitl()
