'''
Copter autotest: StabilityPatch

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class StabilityPatch(TestGroup):
    # StabilityPatch - fly south, then hold loiter within 5m
    # position and altitude and reduce 1 motor to 60% efficiency
    @testcase(bucket="CopterTests1d")
    def StabilityPatch(self,
                       holdtime=30,
                       maxaltchange=5,
                       maxdistchange=10):
        '''Fly stability patch'''
        self.takeoff(10, mode="LOITER")

        # first south
        self.progress("turn south")
        self.set_rc(4, 1580)
        self.wait_heading(180)
        self.set_rc(4, 1500)

        # fly west 80m
        self.set_rc(2, 1100)
        self.wait_distance(80)
        self.set_rc(2, 1500)

        # wait for copter to slow moving
        self.wait_groundspeed(0, 2)

        m = self.assert_receive_message('VFR_HUD')
        start_altitude = m.alt
        start = self.get_location()
        tstart = self.get_sim_time()
        self.progress("Holding loiter at %u meters for %u seconds" %
                      (start_altitude, holdtime))

        # cut motor 1's to efficiency
        self.progress("Cutting motor 1 to 65% efficiency")
        self.set_parameters({
            "SIM_ENGINE_MUL": 0.65,
            "SIM_ENGINE_FAIL": 1 << 0, # motor 1
        })

        while self.get_sim_time_cached() < tstart + holdtime:
            m = self.assert_receive_message('VFR_HUD')
            pos = self.get_location()
            delta = self.get_distance(start, pos)
            alt_delta = math.fabs(m.alt - start_altitude)
            self.progress("Loiter Dist: %.2fm, alt:%u" % (delta, m.alt))
            if alt_delta > maxaltchange:
                raise NotAchievedException(
                    "Loiter alt shifted %u meters (> limit %u)" %
                    (alt_delta, maxaltchange))
            if delta > maxdistchange:
                raise NotAchievedException(
                    ("Loiter shifted %u meters (> limit of %u)" %
                     (delta, maxdistchange)))

        # restore motor 1 to 100% efficiency
        self.set_parameter("SIM_ENGINE_MUL", 1.0)

        self.progress("Stability patch and Loiter OK for %us" % holdtime)

        self.progress("RTL after stab patch")
        self.do_RTL()
