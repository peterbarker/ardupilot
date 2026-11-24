'''
Copter autotest: FenceMarginTypeChange

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class FenceMarginTypeChange(TestGroup):
    @testcase(bucket="CopterTests2c")
    def FenceMarginTypeChange(self):
        '''Test removing a margin-breached fence from FENCE_TYPE in flight'''
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 2,    # circle fence
            "FENCE_RADIUS": 50,
            "FENCE_MARGIN": 5,
            "AVOID_ENABLE": 0,
            "FENCE_OPTIONS": 4,  # notify on margin breach
        })
        self.context_collect('STATUSTEXT')

        self.takeoff(10, mode="GUIDED")
        # inside the circle fence but within its margin:
        self.fly_guided_move_local(47.5, 0, 10)
        self.wait_statustext("Circle fence in", check_context=True)

        self.progress("Replacing the circle fence with a polygon fence")
        self.set_parameter("FENCE_TYPE", 4)
        # only the fence whose margin breach was cleared should be
        # named; the whole complemented FENCE_TYPE mask used to be
        # reported, indexing off the end of the fence names table
        self.wait_statustext("Circle fence cleared margin breach", check_context=True)

        self.do_RTL()
