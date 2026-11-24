'''
QuadPlane autotest: VTOLQuicktune_CPP

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class VTOLQuicktune_CPP(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def VTOLQuicktune_CPP(self):
        '''VTOL Quicktune in C++'''
        self.context_preserve_parameters(self.quicktune_saved_gains)
        self.set_parameters({
            "RC7_OPTION": 181,
            "QWIK_ENABLE" : 1,
            "QWIK_DOUBLE_TIME" : 5, # run faster for autotest
        })

        self.context_push()
        self.context_collect('STATUSTEXT')

        # reduce roll/pitch gains by 2
        gain_mul = 0.5
        soften_params = ['Q_A_RAT_RLL_P', 'Q_A_RAT_RLL_I', 'Q_A_RAT_RLL_D',
                         'Q_A_RAT_PIT_P', 'Q_A_RAT_PIT_I', 'Q_A_RAT_PIT_D',
                         'Q_A_RAT_YAW_P', 'Q_A_RAT_YAW_I']

        original_values = self.get_parameters(soften_params)

        softened_values = {}
        for p in original_values.keys():
            softened_values[p] = original_values[p] * gain_mul
        self.set_parameters(softened_values)

        self.wait_ready_to_arm()
        self.change_mode("QLOITER")
        self.set_rc(7, 1000)
        self.arm_vehicle()
        self.takeoff(20, 'QLOITER')

        # use rc switch to start tune
        self.set_rc(7, 1500)

        self.wait_text("Quicktune: starting tune", check_context=True)
        for axis in ['Roll', 'Pitch', 'Yaw']:
            self.wait_text("Starting %s tune" % axis, check_context=True)
            self.wait_text("Quicktune: %s D done" % axis, check_context=True, timeout=120)
            self.wait_text("Quicktune: %s P done" % axis, check_context=True, timeout=120)
            self.wait_text("Quicktune: %s done" % axis, check_context=True, timeout=120)

        new_values = self.get_parameters(soften_params)
        for p in original_values.keys():
            threshold = 0.8 * original_values[p]
            self.progress("tuned param %s %.4f need %.4f" % (p, new_values[p], threshold))
            if new_values[p] < threshold:
                raise NotAchievedException(
                    "parameter %s %.4f not increased over %.4f" %
                    (p, new_values[p], threshold))

        self.progress("ensure we are not overtuned")
        self.set_parameters({
            'SIM_ENGINE_MUL': 0.9,
            'SIM_ENGINE_FAIL': 1 << 0,
        })

        self.delay_sim_time(5, reason="engine failure effect")

        # and restore it
        self.set_parameter('SIM_ENGINE_MUL', 1)

        for i in range(5):
            self.wait_heartbeat()

        if self.statustext_in_collections("ABORTING"):
            raise NotAchievedException("tune has aborted, overtuned")

        self.progress("using aux fn for save tune")

        # to test aux function method, use aux fn for save
        self.run_auxfunc(181, 2)
        self.wait_text("Quicktune: saved", check_context=True)
        self.change_mode("QLAND")

        self.wait_disarmed(timeout=120)
        self.set_parameter("QWIK_ENABLE", 0)
        self.context_pop()
        self.reboot_sitl()
