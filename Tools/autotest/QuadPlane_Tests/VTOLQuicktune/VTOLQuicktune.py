'''
QuadPlane autotest: VTOLQuicktune

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class VTOLQuicktune(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def VTOLQuicktune(self):
        '''VTOL Quicktune'''
        self.context_preserve_parameters(self.quicktune_saved_gains)
        self.install_applet_script_context("VTOL-quicktune.lua")

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SIM_SPEEDUP": 20, # need to give some cycles to lua
            "RC7_OPTION": 300,
        })

        self.reboot_sitl()

        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "QUIK_ENABLE" : 1,
            "QUIK_DOUBLE_TIME" : 5, # run faster for autotest
            })

        self.scripting_restart()
        self.wait_text("Quicktune for quadplane loaded", check_context=True)

        self.wait_ready_to_arm()
        self.change_mode("QLOITER")
        self.arm_vehicle()
        self.takeoff(20, 'QLOITER')

        # use rc switch to start tune
        self.set_rc(7, 1500)

        self.wait_text("Tuning: starting tune", check_context=True)
        for axis in ['RLL', 'PIT', 'YAW']:
            self.wait_text("Starting %s tune" % axis, check_context=True)
            self.wait_text("Tuning: %s_D done" % axis, check_context=True, timeout=120)
            self.wait_text("Tuning: %s_P done" % axis, check_context=True, timeout=120)
            self.wait_text("Tuning: %s done" % axis, check_context=True, timeout=120)
        self.wait_text("Tuning: YAW done", check_context=True, timeout=120)

        # to test aux function method, use aux fn for save
        self.run_auxfunc(300, 2)
        self.wait_text("Tuning: saved", check_context=True)
        # and put the switch back: aux function state survives
        # context_pop(), and this test does not reboot afterwards
        self.run_auxfunc(300, 0)
        self.change_mode("QLAND")

        self.wait_disarmed(timeout=120)
