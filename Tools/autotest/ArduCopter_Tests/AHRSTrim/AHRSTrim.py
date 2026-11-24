'''
Copter autotest: AHRSTrim

AP_FLAKE8_CLEAN
'''

import math

from test_discovery import TestGroup
from test_discovery import testcase


class AHRSTrim(TestGroup):
    @testcase(bucket="CopterTests2d")
    def AHRSTrim(self):
        '''check RC stick inputs can be used to save an AHRS trim'''
        # the Save Trim procedure requires the throttle to be at minimum
        # ("landed").  The pilot holds the roll and pitch sticks to the
        # desired trim and then raises the aux switch.  Trim is taken from
        # the stick positions, not the vehicle's attitude.
        #
        # save-trim modifies AHRS_TRIM_X/Y directly rather than via a
        # parameter set, so we must explicitly set them to a known
        # (non-zero) starting value here for the harness to record and
        # revert them after the test.  We use the opposite sign to the
        # trim the sticks will command, so the save has to overcome them.
        self.set_parameters({
            'RC9_OPTION': 5,  # save-trim
            'AHRS_TRIM_X': -0.05,  # ensure parameter-reversion
            'AHRS_TRIM_Y': 0.05,  # ensure parameter-reversion
        })
        self.change_mode('STABILIZE')
        self.set_rc_from_map({
            1: 1900,  # roll full right
            2: 1100,  # pitch full forward
            3: 1000,  # throttle to minimum so its control_in is zero
        })
        self.context_collect('STATUSTEXT')
        self.set_rc(9, 2000)
        self.wait_statustext('Trim saved', check_context=True)
        self.context_stop_collecting('STATUSTEXT')

        # lower the switch and restore the sticks so the trim can't be
        # applied a second time (e.g. across the reboot below):
        self.set_rc(9, 1000)
        self.set_rc_default()

        # trim is taken from the stick positions, so right-roll/forward-pitch
        # should overcome the starting trims to give a positive roll trim
        # and a negative pitch trim.  On the ground the pilot's total lean
        # is limited to 10 degrees (the floor on the althold lean angle
        # limit), and the diagonal stick splits that total tilt equally
        # between the axes (see rc_input_to_roll_pitch_rad):
        thrust_per_axis = math.tan(math.radians(10)) / math.sqrt(2)
        stick_pitch = -math.atan(thrust_per_axis)
        stick_roll = math.atan(math.cos(stick_pitch) * thrust_per_axis)
        expected_trims = {
            "AHRS_TRIM_X": -0.05 + stick_roll,
            "AHRS_TRIM_Y": 0.05 + stick_pitch,
        }
        self.assert_parameter_values(expected_trims, epsilon=0.001)

        # the trim is saved to storage, so should survive a reboot:
        self.reboot_sitl()
        self.assert_parameter_values(expected_trims, epsilon=0.001)
