'''
Copter autotest: AC_Avoidance_Fence

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AC_Avoidance_Fence(TestGroup):
    @testcase(bucket="CopterTests1a")
    def AC_Avoidance_Fence(self):
        '''Test fence avoidance slide behaviour'''
        self.load_fence("copter-avoidance-fence.txt")
        self.set_parameter("FENCE_ENABLE", 1)
        self.check_avoidance_corners()
