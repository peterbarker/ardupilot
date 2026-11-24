'''
Rover autotest: JammingSimulation

AP_FLAKE8_CLEAN
'''

import copy

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class JammingSimulation(TestGroup):
    @testcase(bucket="Rover")
    def JammingSimulation(self):
        '''Test jamming simulation works'''
        self.wait_ready_to_arm()
        start_loc = self.assert_receive_message('GPS_RAW_INT')
        self.set_parameter("SIM_GPS1_JAM", 1)

        class Requirement():
            def __init__(self, field, min_value):
                self.field = field
                self.min_value = min_value

            def met(self, m):
                return getattr(m, self.field) > self.min_value

        requirements = set([
            Requirement('v_acc', 50000),
            Requirement('h_acc', 50000),
            Requirement('vel_acc', 1000),
            Requirement('vel', 10000),
        ])
        low_sats_seen = False
        seen_bad_loc = False
        tstart = self.get_sim_time()

        while True:
            if self.get_sim_time() - tstart > 120:
                raise NotAchievedException("Did not see all jamming")
            m = self.assert_receive_message('GPS_RAW_INT')
            new_requirements = copy.copy(requirements)
            for requirement in requirements:
                if requirement.met(m):
                    new_requirements.remove(requirement)
            requirements = new_requirements
            if m.satellites_visible < 6:
                low_sats_seen = True
            here = self.assert_receive_message('GPS_RAW_INT')
            if self.get_distance_int(start_loc, here) > 100:
                seen_bad_loc = True

            if len(requirements) == 0 and low_sats_seen and seen_bad_loc:
                break
