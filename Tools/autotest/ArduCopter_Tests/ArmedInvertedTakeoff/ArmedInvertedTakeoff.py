'''
Copter autotest: ArmedInvertedTakeoff

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ArmedInvertedTakeoff(TestGroup):
    @testcase(bucket="CopterTests1e")
    def ArmedInvertedTakeoff(self):
        '''check raising the throttle on a vehicle armed upside down rights it without an internal error'''
        self.context_collect('STATUSTEXT')

        # turn the simulated vehicle upside down on the ground.  The AHRS
        # must use the simulated attitude, and SIM_GND_BEHAV of 0 stops the
        # simulator levelling a multicopter on the ground
        self.install_test_script_context("sim_set_roll.lua")
        self.set_parameters({
            "SCR_ENABLE": 1,
            "AHRS_EKF_TYPE": 10,
            "EK3_ENABLE": 0,
            "SIM_GND_BEHAV": 0,
        })
        self.reboot_sitl()
        self.delay_sim_time(10, reason="AHRS to start")
        self.set_parameter("SCR_USER1", 180)
        self.wait_roll(180, 20, timeout=30)
        self.delay_sim_time(5, reason="vehicle to settle")
        self.change_mode('STABILIZE')

        # the arming checks would refuse an upside-down vehicle
        self.arm_vehicle(force=True)
        self.wait_roll(180, 45, timeout=2)
        self.set_rc(3, 1500)
        self.wait_roll(0, 20, timeout=20)
        self.set_rc(3, 1000)
        if self.armed():
            self.disarm_vehicle(force=True)

        if self.statustext_in_collections("Internal Errors"):
            raise NotAchievedException("Internal error raised while armed inverted")
