'''
Plane autotest: ScriptedArmingChecksAppletEStop

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptedArmingChecksAppletEStop(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def ScriptedArmingChecksAppletEStop(self):
        """ Applet for Arming Checks will prevent a vehicle from arming based on scripted checks
            """
        self.start_subtest("Scripted Arming Checks Applet validation")
        self.context_collect("STATUSTEXT")

        """Initialize the FC"""
        self.set_parameter("SCR_ENABLE", 1)
        self.install_applet_script_context("arming-checks.lua")
        self.reboot_sitl()
        self.wait_ekf_happy()
        self.wait_text("ArduPilot Ready", check_context=True)
        self.wait_text("Arming Checks .* loaded", timeout=30, check_context=True, regex=True)
        self.set_parameters({
            "ARM_SYSID": -1,
            "ARM_FOLL_SYSID": -1,
            "ARM_FOLL_SYSID_X": -1,
            "ARM_FOLL_OFS_DEF": -1,
            "ARM_P_SCALING": -1,
        })

        self.start_subsubtest("ArmCk: Cannot arm while motors estopped")
        self.set_parameter("RC6_OPTION", 165)
        self.progress("rebooting to enable RC channel")
        self.reboot_sitl() # to handle RC option changing
        self.wait_ekf_happy()
        self.wait_text("ArduPilot Ready", check_context=True)
        self.wait_text("Arming Checks .* loaded", timeout=30, check_context=True, regex=True)
        self.set_parameters({
            "ARM_SYSID": -1,
            "ARM_FOLL_SYSID": -1,
            "ARM_FOLL_SYSID_X": -1,
            "ARM_FOLL_OFS_DEF": -1,
            "ARM_P_SCALING": -1,
        })
        self.set_rc(6, 1000)
        self.assert_prearm_failure("ArmCk: fail: Motors EStopped", other_prearm_failures_fatal=False)
        self.set_rc(6, 2000)
        self.wait_text("clear: Motors EStopped", timeout=30, check_context=True, regex=True)
        self.wait_ready_to_arm()
