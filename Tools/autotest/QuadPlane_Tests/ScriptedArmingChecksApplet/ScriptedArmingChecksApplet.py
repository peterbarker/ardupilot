'''
QuadPlane autotest: ScriptedArmingChecksApplet

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptedArmingChecksApplet(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def ScriptedArmingChecksApplet(self):
        """ Applet for Arming Checks will prevent a vehicle from arming based on scripted checks
            """
        self.start_subtest("Scripted Arming Checks Applet validation")
        self.context_collect("STATUSTEXT")

        applet_script = "arming-checks.lua"
        """Initialize the FC"""
        self.set_parameter("SCR_ENABLE", 1)
        self.install_applet_script_context(applet_script)
        self.reboot_sitl()
        self.wait_ekf_happy()
        self.wait_text("ArduPilot Ready", check_context=True)
        self.wait_text("Arming Checks .* loaded", timeout=30, check_context=True, regex=True)
        '''self.install_messageprinter_handlers_context(['PARAM_VALUE'])'''

        self.start_subsubtest("ArmCk: Q_RTL_ALT must be legal")
        self.set_parameter("SCALING_SPEED", 22)
        self.set_parameter("Q_RTL_ALT", 150)
        self.assert_prearm_failure("ArmCk: fail: Q_RTL_ALT too high", other_prearm_failures_fatal=False)
        self.set_parameter("Q_RTL_ALT", 120)
        self.wait_text("clear: Q_RTL_ALT", check_context=True)

        self.start_subsubtest("ArmCk: Q_RTL vs QLand")
        ''' this is only a warning'''
        self.set_parameter("Q_OPTIONS", 33)
        self.wait_text("ArmCk: note: Q will RTL", check_context=True)
        self.set_parameter("Q_OPTIONS", 1)
        self.wait_text("ArmCk: note: Q will land", check_context=True)
