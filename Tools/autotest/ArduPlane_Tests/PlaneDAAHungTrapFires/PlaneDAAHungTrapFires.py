'''
Plane autotest: PlaneDAAHungTrapFires

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAHungTrapFires(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAHungTrapFires(self):
        '''Avoidance that is running but making no progress must be detected, alerted and
        trapped - and released again when the mission moves on.

        The trapped-failsafe only ever fired on a sustained fence breach or an aircraft
        near-miss.  A vehicle avoiding correctly - never breaching, no traffic anywhere -
        but unable to close on its waypoint slipped through entirely: it orbited until
        fuel ran out, or until it eventually clipped the fence it had been dodging and the
        autopilot's own FENCE_ACTION caught it.  Seen in flight on T2 Cruza log_77/83/84
        and Niska log_104, and reproduced in SITL.

        The geometry is log_77's: waypoint 2 sits just inside the fence standoff, where the
        mission pull and the fence push are irreconcilable.  A 300 m exclusion circle with
        DAA_MARGIN_FENCE=150 gives a 450 m keep-out; the waypoint is 350 m from the centre,
        so it is 100 m inside the standoff but still 50 m OUTSIDE the fence itself.  That
        matters: the vehicle is never boxed in and never has to breach anything, it simply
        stops at 450 m and can never close the last 100 m.  (A waypoint placed DEEP inside
        the keep-out is not a stall - the sweep runs out of clear headings, falls through to
        the best blocked one, flies in and arrives.)  WP_RADIUS is pinned small so waypoint
        acceptance cannot paper over the shortfall.

        DAA_HUNG_ALRT_S must notice, DAA_TRAP_ACT must fire with the cause reported as
        "hung", and advancing the mission to a reachable waypoint must release it - progress
        toward the target cannot be the release test, because the trap's own RTL retargets
        the vehicle to home and would read as instant progress.'''
        self._PlaneDAAHungTrapFires(alert_only=False)
