'''
Copter autotest: FTPScriptUpload

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FTPScriptUpload(TestGroup):
    @testcase(bucket="CopterTests2c")
    def FTPScriptUpload(self):
        '''upload a Lua script via MAVLink FTP and verify it runs'''
        # SCRIPTING_DIRECTORY in SITL is "./scripts" (lua_common_defs.h)
        script_path = "scripts/ftp_upload_test.lua"
        script_content = b"gcs:send_text(6, 'ftp_upload_test: ok')\n"

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 20000,
        })
        self.reboot_sitl()

        try:
            self.ftp_create_directory("scripts")
        except NotAchievedException:
            pass

        try:
            self.ftp_write_file(script_path, script_content)
            self.context_collect('STATUSTEXT')
            self.scripting_restart()
            self.wait_statustext(
                "ftp_upload_test: ok",
                check_context=True,
                timeout=30,
                wallclock_timeout=True,
            )
        except Exception:
            try:
                self.ftp_remove_file(script_path)
            except NotAchievedException:
                pass
            raise
        self.ftp_remove_file(script_path)
