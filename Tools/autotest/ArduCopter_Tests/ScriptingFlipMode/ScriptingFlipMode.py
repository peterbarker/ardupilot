'''
Copter autotest: ScriptingFlipMode

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptingFlipMode(TestGroup):
    @testcase(bucket="CopterTests1d")
    def ScriptingFlipMode(self):
        '''test adding custom mode from scripting'''
        # Really it would be nice to check for the AVAILABLE_MODES message, but pymavlink does not understand them yet.

        # enable scripting and install flip script
        self.set_parameters({
            "SCR_ENABLE": 1,
        })
        self.install_example_script_context('Flip_Mode.lua')
        self.reboot_sitl()

        # Takeoff in loiter
        self.takeoff(10, mode="LOITER")

        # Try and switch to flip, should not be possible from loiter
        try:
            self.change_mode(100, timeout=10)
        except AutoTestTimeoutException:
            self.progress("PASS not able to enter from loiter")

        # Should be alowd to enter from alt hold
        self.change_mode("ALT_HOLD")
        self.change_mode(100)

        # Should return to previous mode after flipping
        self.wait_mode("ALT_HOLD")

        # Test done
        self.land_and_disarm()
