'''
Copter autotest: PosHoldDesiredAttitudeMonotonic

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import quaternion

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PosHoldDesiredAttitudeMonotonic(TestGroup):
    @testcase(bucket="CopterTests2b")
    def PosHoldDesiredAttitudeMonotonic(self):
        '''check PosHold desired attitude does not reverse during a sustained stick input
        '''

        '''
        Reproduces https://github.com/ArduPilot/ardupilot/issues/33588 : starting from
        a settled PosHold/LOITER hold, a single sustained roll or pitch stick input
        produces a non-monotonic, two-stage desired-attitude response - the desired
        attitude moves in the commanded direction, briefly reverses back towards its
        previous value, then resumes towards the commanded attitude.  Copter-4.6.3
        produces a single smooth response.  The desired attitude here should rise
        essentially monotonically while the stick is held.
        '''
        # the size of desired-attitude reversal (deg) we are prepared to tolerate.
        # a correct (monotonic) response stays well under this; the regression
        # produces a reversal of roughly 8deg.
        max_allowed_reversal_deg = 3.0

        self.set_message_rate_hz('ATTITUDE_TARGET', 50)
        self.takeoff(15, mode='GUIDED')
        self.set_rc(3, 1500)
        self.change_mode('POSHOLD')

        for (name, channel, axis) in [("roll", 1, 0), ("pitch", 2, 1)]:
            # let PosHold brake and settle into a stationary LOITER hold so the
            # desired attitude starts near zero before the step input.  The
            # PILOT_OVERRIDE->BRAKE->LOITER transition is time-driven and not
            # observable over MAVLink, so settle on a fixed delay and then confirm
            # the vehicle has actually come to rest:
            self.progress(f"Settling into LOITER hold before {name} step")
            self.delay_sim_time(8, reason="braking to a stationary LOITER hold before step input")
            self.wait_groundspeed(0, 0.5, minimum_duration=2, timeout=30)

            # sample the desired attitude continuously, applying a sustained
            # full-deflection step part way through so we capture the response
            # from before the step until well after it has settled:
            tstart = self.get_sim_time()
            samples = []
            applied = False
            while self.get_sim_time_cached() - tstart < 3.0:
                m = self.assert_receive_message('ATTITUDE_TARGET')
                t = self.get_sim_time_cached() - tstart
                q = quaternion.Quaternion([m.q[0], m.q[1], m.q[2], m.q[3]])
                samples.append(math.degrees(q.euler[axis]))
                if not applied and t > 0.5:
                    self.progress("Applying sustained %s input" % name)
                    self.set_rc(channel, 1900)
                    applied = True
            self.set_rc(channel, 1500)

            # the response is commanded positive; find the worst reversal (a fall
            # back towards zero after the desired attitude has begun to rise):
            peak = 0.0
            max_reversal = 0.0
            for desired in samples:
                if desired > peak:
                    peak = desired
                elif peak > 5 and (peak - desired) > max_reversal:
                    max_reversal = peak - desired
            self.progress("%s: desired-attitude peak=%.1fdeg reversal=%.1fdeg" %
                          (name, peak, max_reversal))
            if max_reversal > max_allowed_reversal_deg:
                raise NotAchievedException(
                    "PosHold %s desired attitude reversed by %.1fdeg (>%.1f) during "
                    "sustained input (issue #33588)" %
                    (name, max_reversal, max_allowed_reversal_deg))

        self.do_RTL()
