'''
Copter autotest: HorizontalAvoidFence

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class HorizontalAvoidFence(TestGroup):
    @testcase(bucket="CopterTests1c")
    def HorizontalAvoidFence(self, timeout=180):
        '''Test horizontal Avoidance fence'''
        self.fly_fence_avoid_test_radius_check(avoid_behave=1, timeout=timeout)
        self.fly_fence_avoid_test_radius_check(avoid_behave=0, timeout=timeout)
