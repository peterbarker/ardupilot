'''
Rover autotest: JSONClock

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class JSONClock(TestGroup):
    @testcase(bucket="Rover")
    def JSONClock(self):
        '''check SITL time follows JSON physics time without drift and across a physics reset'''
        # wall-clock physics time, with a step SITL used to truncate by 1us each frame
        start_time = 1760000000.0
        with self.JSON_physics(start_time=start_time, dt=0.0009999995) as physics:
            self.wait_heartbeat()

            # SITL time counts from the first frame, without drift
            first = (0, start_time, start_time)
            self.JSON_assert_elapsed_equal(first, self.JSON_sim_and_physics_time(physics))
            self.delay_sim_time(200, reason="clock drift")
            self.JSON_assert_elapsed_equal(first, self.JSON_sim_and_physics_time(physics))

            # the physics restarts its clock, SITL time carries on following it
            before_reset = self.JSON_sim_and_physics_time(physics)
            with physics.lock:
                physics.timestamp = 1.0
            after_reset = self.JSON_sim_and_physics_time(physics)
            if after_reset[0] < before_reset[0]:
                raise NotAchievedException("SITL time went back on physics reset")
            self.delay_sim_time(20, reason="after physics reset")
            self.JSON_assert_elapsed_equal(after_reset, self.JSON_sim_and_physics_time(physics))
