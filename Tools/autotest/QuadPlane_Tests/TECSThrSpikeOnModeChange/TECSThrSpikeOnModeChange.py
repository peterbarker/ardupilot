'''
QuadPlane autotest: TECSThrSpikeOnModeChange

AP_FLAKE8_CLEAN
'''

import vehicle_test_suite

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TECSThrSpikeOnModeChange(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def TECSThrSpikeOnModeChange(self):
        ''' Regression test for issue #33871. '''

        # The bug only affects vectored tiltrotors.
        self.customise_SITL_commandline(
            [],
            model="quadplane-tilthvec",
            wipe=True,
        )

        # Turn off throttle slew limit
        self.set_parameter("THR_SLEWRATE", 0)

        # Take off and transition to fixed-wing flight.
        self.takeoff(25, mode='QHOVER', timeout=120)
        self.context_collect('STATUSTEXT')
        self.change_mode('FBWA')
        self.set_rc(3, 2000)
        self.wait_statustext('Transition FW done', timeout=60)

        # Add hook to check throttle level
        class DetectThrottleSpike(vehicle_test_suite.TestSuite.MessageHook):
            '''Checks for spikes in throttle output'''
            def __init__(self, suite):
                super(DetectThrottleSpike, self).__init__(suite)
                self.num_samples = 0

            def hook_removed(self):
                if self.num_samples == 0:
                    raise NotAchievedException("Did not get SERVO_OUTPUT_RAW")

            def process(self, mav, m):
                if m.get_type() != 'SERVO_OUTPUT_RAW':
                    return

                self.num_samples += 1
                if m.servo3_raw > 1750:
                    raise NotAchievedException("Throttle spike (%u)" % (m.servo3_raw))

        self.set_message_rate_hz('SERVO_OUTPUT_RAW', 200)

        # Install a hook to check for throttle spike
        self.context_push()
        self.install_message_hook_context(DetectThrottleSpike(self))

        # Fly in CRUISE so TECS runs and _throttle_dem converges toward 80%
        # (TRIM_THROTTLE feed-forward keeps _throttle_dem well above 40%).
        self.set_rc(3, 1000)
        self.delay_sim_time(1, reason="Allow vehicle to stabilize at low throttle")
        self.change_mode('CRUISE')

        self.delay_sim_time(5, reason="Check throttle output")
        self.context_pop()

        self.do_RTL()
