'''
QuadPlane autotest: RudderArmingWithARMING_CHECK_THROTTLEUnset

AP_FLAKE8_CLEAN
'''

import vehicle_test_suite

from test_discovery import TestGroup
from test_discovery import testcase


class RudderArmingWithARMING_CHECK_THROTTLEUnset(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def RudderArmingWithARMING_CHECK_THROTTLEUnset(self) -> None:
        '''check arming behaviour with ARMING_CHECK_THROTTLE unset'''
        self.wait_ready_to_arm()

        self.start_subtest("Should not be able to arm with mid-stick throttle")
        self.set_rc_from_map({3: 1500, 4: 2000})
        w = vehicle_test_suite.WaitAndMaintainDisarmed(self, minimum_duration=10)
        w.run()
        self.set_rc(4, 1500)
        self.disarm_vehicle()

        self.clear_parameter_bit("RC_OPTIONS", 5)
        self.start_subtest("Should be able to arm with mid-stick throttle")
        self.set_rc_from_map({3: 1500, 4: 2000})
        self.wait_armed()
        self.set_rc(4, 1500)
        self.disarm_vehicle()
