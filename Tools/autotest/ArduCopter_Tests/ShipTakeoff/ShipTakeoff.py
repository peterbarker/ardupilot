'''
Copter autotest: ShipTakeoff

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ShipTakeoff(TestGroup):
    @testcase(bucket="CopterTests2d")
    def ShipTakeoff(self):
        '''Fly Simulated Ship Takeoff'''
        # test ship takeoff
        self.wait_groundspeed(0, 2)
        self.set_parameters({
            "SIM_SHIP_ENABLE": 1,
            "SIM_SHIP_SPEED": 10,
            "SIM_SHIP_DSIZE": 2,
        })
        self.wait_ready_to_arm()
        # we should be moving with the ship
        self.wait_groundspeed(9, 11)
        self.takeoff(10)
        # above ship our speed drops to 0
        self.wait_groundspeed(0, 2)
        self.land_and_disarm()
        # ship will have moved on, so we land on the water which isn't moving
        self.wait_groundspeed(0, 2)

        # we are not at the home location - reboot so the next test starts there
        self.set_parameter("SIM_SHIP_ENABLE", 0)
        self.reboot_sitl()
