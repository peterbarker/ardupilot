'''
Rover autotest: JSONNoLockstep

AP_FLAKE8_CLEAN
'''

import time

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class JSONNoLockstep(TestGroup):
    @testcase(bucket="Rover")
    def JSONNoLockstep(self):
        '''check SITL follows a free-running JSON physics backend using no_lockstep'''
        with self.JSON_physics(dt=0.005, free_run=True, fields={'no_lockstep': True}) as physics:
            self.wait_heartbeat()

            # the physics paces SITL, in real time whatever the speedup
            ratio = self.JSON_sim_wall_ratio()
            if not 0.5 < ratio < 1.5:
                raise NotAchievedException("SITL ran at %fx, expected the physics real time" % ratio)
            self.JSON_assert_elapsed_equal((0, 0, 0), self.JSON_sim_and_physics_time(physics))

            # frame rate is not forced to the physics step
            sim_rate = self.get_parameter("SIM_RATE_HZ")
            frame_rate = self.JSON_frame_rate(physics)
            if frame_rate != sim_rate:
                raise NotAchievedException("frame_rate %.1f, expected SIM_RATE_HZ %u" % (frame_rate, sim_rate))

            # SITL keeps running its loop without physics.  Its own frame
            # is made longer than the physics step, which the physics has
            # to win back once it resumes
            self.set_parameter("SIM_RATE_HZ", 100)
            physics.paused = True
            packets = physics.packets
            time.sleep(2)
            sent = physics.packets - packets
            physics.paused = False
            if sent < 10:
                raise NotAchievedException("SITL sent only %u frames without physics" % sent)
            self.wait_heartbeat()

            # after the pause SITL time follows the physics again
            resumed = self.JSON_sim_and_physics_time(physics)
            self.delay_sim_time(4, reason="physics resumed")
            self.JSON_assert_elapsed_equal(resumed, self.JSON_sim_and_physics_time(physics))
