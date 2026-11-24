'''
Plane autotest: PlaneFollowAppletSanity

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneFollowAppletSanity(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def PlaneFollowAppletSanity(self):
        '''PLane Follow Sanity Check, not a detailed test'''
        self.start_subtest("Plane Follow Script Load and Start")

        self.install_applet_script_context("plane_follow.lua")
        self.install_script_module_context(self.script_modules_source_path("pid.lua"), "pid.lua")
        self.install_script_module_context(self.script_modules_source_path("mavlink_attitude.lua"), "mavlink_attitude.lua")
        self.install_mavlink_module_context()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SIM_SPEEDUP": 20, # need to give some cycles to lua
            "RC7_OPTION": 301,
        })

        self.context_collect("STATUSTEXT")

        self.reboot_sitl()

        self.wait_text("Plane Follow .* script loaded", timeout=30, regex=True, check_context=True)

        self.wait_ready_to_arm()
        self.set_rc(7, 2000)
        self.wait_text("PFollow: must be armed", check_context=True)
        self.set_rc(7, 1000)
        self.arm_vehicle()
        self.set_rc(7, 2000)
        self.wait_text("PFollow: enabled", check_context=True)
        self.set_rc(7, 1000)
        self.wait_text("PFollow: disabled", check_context=True)
        self.disarm_vehicle()

        self.reboot_sitl()
