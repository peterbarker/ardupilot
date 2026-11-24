'''
Copter autotest: ScriptMountPOI

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptMountPOI(TestGroup):
    @testcase(bucket="CopterTests1a")
    def ScriptMountPOI(self):
        '''test the MountPOI example script'''
        self.context_push()

        self.install_terrain_handlers_context()
        self.set_parameters({
            "SCR_ENABLE": 1,
            "RC12_OPTION": 300,
        })
        self.setup_servo_mount()
        self.reboot_sitl()
        self.set_rc(6, 1300)
        self.install_applet_script_context('mount-poi.lua')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.context_collect('STATUSTEXT')
        self.set_rc(12, 2000)
        self.wait_statustext('POI.*-35.*149', check_context=True, regex=True)
        self.set_rc(12, 1000)

        self.context_pop()
        self.reboot_sitl()
