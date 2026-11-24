'''
Copter autotest: ScriptingAHRSSource

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptingAHRSSource(TestGroup):
    @testcase(bucket="CopterTests2a")
    def ScriptingAHRSSource(self):
        '''test ahrs-source.lua script'''
        self.install_example_script_context("ahrs-source.lua")
        self.set_parameters({
            "RC10_OPTION": 90,
            "SCR_ENABLE": 1,
            "SCR_USER1": 10,    # something else
            "SCR_USER2": 10,    # GPS something
            "SCR_USER3": 0.2,   # ExtNav innovation
        })
        self.set_rc(10, 2000)
        self.reboot_sitl()
        self.context_collect('STATUSTEXT')
        self.set_rc(10, 1000)
        self.wait_statustext('Using EKF Source Set 1', check_context=True)
        self.set_rc(10, 1500)
        self.wait_statustext('Using EKF Source Set 2', check_context=True)
        self.set_rc(10, 2000)
        self.wait_statustext('Using EKF Source Set 3', check_context=True)
