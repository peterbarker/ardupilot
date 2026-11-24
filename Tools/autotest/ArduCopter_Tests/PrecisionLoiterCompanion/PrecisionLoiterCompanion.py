'''
Copter autotest: PrecisionLoiterCompanion

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PrecisionLoiterCompanion(TestGroup):
    @testcase(bucket="CopterTests2d")
    def PrecisionLoiterCompanion(self):
        """Use Companion PrecLand backend precision messages to loiter."""

        self.set_parameters({
            "PLND_ENABLED": 1,
            "PLND_TYPE": 1,  # enable companion backend:
            "RC7_OPTION": 39,  # set up a channel switch to enable precision loiter:
        })
        self.set_analog_rangefinder_parameters()
        self.reboot_sitl()

        self.progress("Waiting for location")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()

        # we should be doing precision loiter at this point
        start = self.assert_receive_message('LOCAL_POSITION_NED')

        self.takeoff(20, mode='ALT_HOLD')

        # move away a little
        self.set_rc(2, 1550)
        self.wait_distance(5, accuracy=1)
        self.set_rc(2, 1500)
        self.change_mode('LOITER')

        # turn precision loiter on:
        self.context_collect('STATUSTEXT')
        self.set_rc(7, 2000)

        # try to drag aircraft to a position 5 metres north-east-east:
        self.precision_loiter_to_pos(start.x + 5, start.y + 10, start.z + 10, True)
        self.wait_statustext("PrecLand: Target Found", check_context=True, timeout=10)
        self.wait_statustext("PrecLand: Init Complete", check_context=True, timeout=10)
        # .... then northwest
        self.precision_loiter_to_pos(start.x + 5, start.y - 10, start.z + 10, True)

        # repeat using earth-frame targets
        self.precision_loiter_to_pos(start.x + 5, start.y - 10, start.z + 10, False)

        self.disarm_vehicle(force=True)
