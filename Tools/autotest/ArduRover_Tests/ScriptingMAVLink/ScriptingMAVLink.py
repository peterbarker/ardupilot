'''
Rover autotest: ScriptingMAVLink

AP_FLAKE8_CLEAN
'''

import os
import shutil

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptingMAVLink(TestGroup):
    @testcase(bucket="Rover")
    def ScriptingMAVLink(self):
        """Exercise MAVLink layouts and script replies in the embedded Lua interpreter."""
        self.context_push()
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_HEAP_SIZE": 1024000,
            "SCR_VM_I_COUNT": 1000000,
        })

        # Exercise the checked-in module used by these applets, rather than
        # generating a fresh module from the MAVLink submodule.
        self.context_get().installed_modules.append("MAVLink")
        shutil.copytree(self.script_modules_source_path("MAVLink"),
                        self.installed_script_module_path("MAVLink"))
        # Keep the applets outside scripts/ so AP does not execute them directly.
        for source in [
                self.script_example_source_path("MAVLink_Commands.lua"),
                self.script_example_source_path("BQ40Z_bms_shutdown.lua"),
                self.script_applet_source_path("param-lockdown.lua"),
        ]:
            self.install_script_module_context(source, os.path.basename(source))
        self.install_test_scripts_context(["mavlink_layout.lua", "mavlink_replies.lua"])
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        for success_text in [
                "MAVLink layout and target tests passed",
                "MAVLink script reply tests passed",
        ]:
            self.wait_statustext(success_text, check_context=True)
        self.context_pop()
        self.reboot_sitl()
