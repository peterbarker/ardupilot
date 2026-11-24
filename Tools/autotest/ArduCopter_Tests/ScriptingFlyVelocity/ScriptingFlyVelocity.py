'''
Copter autotest: ScriptingFlyVelocity

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptingFlyVelocity(TestGroup):
    @testcase(bucket="CopterTests1e")
    def ScriptingFlyVelocity(self):
        '''test flying a velocity vector using scripting'''

        # enable scripting and install set-target-velocity script
        self.set_parameters({
            "SCR_ENABLE": 1,
        })
        self.install_example_script_context('set-target-velocity.lua')
        self.reboot_sitl()

        # Should be allowed to enter from alt hold
        self.change_mode("ALT_HOLD")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(6, 2000)

        # Should return to previous mode after flipping
        self.wait_mode("RTL", timeout=120)

        # Test done
        self.land_and_disarm()
