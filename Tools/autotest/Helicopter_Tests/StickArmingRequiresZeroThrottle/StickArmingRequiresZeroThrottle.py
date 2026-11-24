'''
Helicopter autotest: StickArmingRequiresZeroThrottle

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class StickArmingRequiresZeroThrottle(TestGroup):
    def assert_not_stick_armed(self, timeout=10):
        '''raise if the vehicle stick-arms within timeout seconds'''
        arming_channel = self.get_stick_arming_channel()
        self.set_output_to_max(arming_channel)
        tstart = self.get_sim_time()
        try:
            while self.get_sim_time_cached() - tstart < timeout:
                if self.armed(poll=False):
                    raise NotAchievedException("Stick-armed when it should not have")
        finally:
            self.set_output_to_trim(arming_channel)

    @testcase(bucket="Helicopter")
    def StickArmingRequiresZeroThrottle(self):
        '''check that stick (rudder) arming requires the collective at zero'''

        '''
        Reproduces https://github.com/ArduPilot/ardupilot/issues/33386 :
        a heli could be stick-armed with the collective/throttle stick
        raised off the bottom stop, a change in behaviour from 4.6 and
        prior.  Stick arming must require zero throttle.
        '''

        # test in stabilize mode with rotor interlock disabled
        self.change_mode('STABILIZE')
        self.set_rc(8, 1000)

        # check arming is possible with collective at zero
        self.start_subtest("Stick arming succeeds with collective at zero")
        self.set_parameter("RC_OPTIONS", 32) # enable Arming check throttle for 0 input
        self.zero_throttle()
        self.wait_ready_to_arm()
        self.arm_motors_with_rc_input()
        self.disarm_vehicle()

        # check arming fails with collective raised
        self.start_subtest("Stick arming is refused with collective raised")
        self.set_rc(3, 1300)
        self.assert_not_stick_armed()

        # check arming succeeds with RC_OPTIONS arming check disabled
        self.start_subtest("Stick arming succeeds with collective raised")
        self.set_parameter("RC_OPTIONS", 0)
        self.set_rc(3, 1300)
        self.arm_motors_with_rc_input()
        self.disarm_vehicle()
