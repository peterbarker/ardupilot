'''
Copter autotest: GuidedYawRate

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GuidedYawRate(TestGroup):
    def do_yaw_rate(self, yaw_rate):
        '''yaw aircraft in guided/rate mode'''
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            p1=60,  # target angle
            p2=0,  # degrees/second
            p3=1,  # -1 is counter-clockwise, 1 clockwise
            p4=1,  # 1 for relative, 0 for absolute
            quiet=True,
        )

    @testcase(bucket="CopterTests1a")
    def GuidedYawRate(self):
        '''ensuer guided yaw rate is not affected by rate of sewt-attitude messages'''
        self.takeoff(30, mode='GUIDED')
        rates = {}
        for rate in 1, 10:
            # command huge yaw rate for a while
            tstart = self.get_sim_time()
            interval = 1/rate
            yawspeed_rads_sum = 0
            yawspeed_rads_count = 0
            last_sent = 0
            while True:
                self.drain_mav()
                tnow = self.get_sim_time_cached()
                if tnow - last_sent > interval:
                    self.do_yaw_rate(60)  # this is... unlikely
                    last_sent = tnow
                if tnow - tstart < 5:  # let it spin up to speed first
                    continue
                yawspeed_rads_sum += self.mav.messages['ATTITUDE'].yawspeed
                yawspeed_rads_count += 1
                if tnow - tstart > 15:  # 10 seconds of measurements
                    break
            yawspeed_degs = math.degrees(yawspeed_rads_sum / yawspeed_rads_count)
            rates[rate] = yawspeed_degs
            self.progress("Input rate %u hz: average yaw rate %f deg/s" % (rate, yawspeed_degs))

        if rates[10] < rates[1] * 0.95:
            raise NotAchievedException("Guided yaw rate slower for higher rate updates")

        self.do_RTL()
