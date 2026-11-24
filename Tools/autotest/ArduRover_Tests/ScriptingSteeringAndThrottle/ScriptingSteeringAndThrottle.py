'''
Rover autotest: ScriptingSteeringAndThrottle

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptingSteeringAndThrottle(TestGroup):
    @testcase(bucket="Rover")
    def ScriptingSteeringAndThrottle(self):
        '''Scripting test - steering and throttle'''
        self.start_subtest("Scripting square")

        self.context_push()
        self.install_example_script_context("rover-set-steering-and-throttle.lua")
        self.set_parameter("SCR_ENABLE", 1)
        self.reboot_sitl()

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(6, 2000)
        tstart = self.get_sim_time()
        while not self.mode_is("HOLD", poll=False):
            if self.get_sim_time_cached() - tstart > 30:
                raise NotAchievedException("Did not move to hold")
            m = self.assert_receive_message('VFR_HUD')
            if m is not None:
                self.progress("Current speed: %f" % m.groundspeed)
        self.disarm_vehicle()

        self.context_pop()
        self.reboot_sitl()
