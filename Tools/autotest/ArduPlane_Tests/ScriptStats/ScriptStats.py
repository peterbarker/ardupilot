'''
Plane autotest: ScriptStats

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptStats(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def ScriptStats(self):
        '''test script stats logging'''
        self.context_push()
        self.set_parameters({
            'SCR_ENABLE': 1,
            'SCR_DEBUG_OPTS': 8,  # runtime memory usage and time
        })
        self.install_test_scripts_context([
            "math.lua",
            "strings.lua",
        ])
        self.install_example_script_context('simple_loop.lua')
        self.context_collect('STATUSTEXT')

        self.reboot_sitl()

        self.wait_statustext('hello, world')
        delay = 20
        self.delay_sim_time(delay, reason='gather some stats')
        self.wait_statustext("math.lua exceeded time limit", check_context=True, timeout=0)

        dfreader = self.dfreader_for_current_onboard_log()
        seen_hello_world = False
#        runtime = None
        while True:
            m = dfreader.recv_match(type=['SCR'])
            if m is None:
                break
            if m.Name == "simple_loop.lua":
                seen_hello_world = True
#            if m.Name == "math.lua":
#                runtime = m.Runtime

        if not seen_hello_world:
            raise NotAchievedException("Did not see simple_loop.lua script")

#        self.progress(f"math took {runtime} seconds to run over {delay} seconds")
#        if runtime == 0:
#            raise NotAchievedException("Expected non-zero runtime for math")

        self.context_pop()
        self.reboot_sitl()
