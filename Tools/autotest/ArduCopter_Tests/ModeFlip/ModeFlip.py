'''
Copter autotest: ModeFlip

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

import vehicle_test_suite

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ModeFlip(TestGroup):
    @testcase(bucket="CopterTests2c")
    def ModeFlip(self, do_pitch_flip=True):
        '''Fly Flip Mode'''
        class WatchForMode(vehicle_test_suite.TestSuite.MessageHook):
            """Records whether specified mode was ever entered."""
            def __init__(self, suite: vehicle_test_suite.TestSuite, mode: str):
                super(WatchForMode, self).__init__(suite)
                self.desired_mode = suite.get_mode_from_mode_mapping(mode)
                self.mode_was_observed = False

            def process(self, mav, m):
                if self.mode_was_observed:
                    return
                if m.get_type() == 'HEARTBEAT' and m.custom_mode == self.desired_mode:
                    self.mode_was_observed = True

        self.context_set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100)
        pre_flip_altitude_m = 20.0

        def get_ready_for_flip():
            """Ensure copter gets to the proper preconditions to test a flip."""
            self.progress("Regaining altitude")
            self.change_alt(pre_flip_altitude_m, threshold_below_m=1.0, threshold_above_m=1.0)

        acceptable_altitude_loss_during_flip_m = 10.0
        self.takeoff(pre_flip_altitude_m, mode='GUIDED')
        self.hover()
        self.change_mode('ALT_HOLD')

        get_ready_for_flip()
        self.start_subtest("Test 1: default flip (roll-right) initialized by RC Aux.")

        def wait_for_roll_right_flip():
            """These checkpoints and thresholds need hand-tuning to avoid flakes.

            In the future, they may be replaced by a lower-overhead solution,
            such as a MessageHook + logic to analyze the observed values to find
            if the flip happened or not.
            """
            self.wait_attitude(despitch=0, desroll=120, tolerance=30)
            self.wait_attitude(despitch=0, desroll=-120, tolerance=30)
            self.wait_attitude(despitch=0, desroll=-30, tolerance=30)
            self.wait_attitude(despitch=0, desroll=0, tolerance=30)

        flip_mode_watcher = WatchForMode(self, 'FLIP')
        self.install_message_hook(flip_mode_watcher)
        neutral_stick = 1500
        self.set_rc(1, neutral_stick) # forces default flip behavior
        flip_rc_aux_channel = 9
        aux_low = 1000
        aux_high = 2000
        self.set_rc(flip_rc_aux_channel, aux_low)
        rc_option_flip_mode = 2 # (source: RC1_OPTION definition)
        self.set_parameter(f'RC{flip_rc_aux_channel}_OPTION', rc_option_flip_mode)
        self.set_rc(flip_rc_aux_channel, aux_high, timeout=None)
        try:
            wait_for_roll_right_flip()
        except AutoTestTimeoutException:
            raise NotAchievedException("Flip not confirmed. (If the flip did happen, our detection needs tuning.)")
        self.progress("Waiting for level")
        self.wait_attitude(despitch=0, desroll=0, tolerance=5)
        self.wait_mode('ALT_HOLD')
        if not flip_mode_watcher.mode_was_observed:
            raise NotAchievedException("Flip mode did not occur as expected.")
        self.remove_message_hook(flip_mode_watcher)
        self.wait_altitude(
            pre_flip_altitude_m - acceptable_altitude_loss_during_flip_m,
            pre_flip_altitude_m + acceptable_altitude_loss_during_flip_m,
            relative=True,
        )

        if do_pitch_flip:
            self.start_subtest("Test 2: flip in a pilot-directed direction initialized by mode-change to FLIP.")
            get_ready_for_flip()

            def wait_for_pitch_back_flip():
                """These checkpoints and thresholds need hand-tuning to avoid flakes.

                Due to Euler-angle choices, a pitch-back flip progresses like:
                  Roll = 0, pitch 0 -> +90
                  (instantly with pitch = +90) Roll 0 -> 180
                  Roll = 180, pitch +90 -> 0 -> -90
                  (instantly with pitch = -90) Roll 180 -> 0
                  Roll = 0, pitch -90 -> 0
                  (Note that Roll ~180 might be positive or negative, or even flip-flop)

                In the future, the detection logic may be replaced by a lower-overhead solution,
                such as a MessageHook + logic to analyze the observed values to find
                if the flip happened or not.
                """
                self.wait_attitude(despitch=60, desroll=0, tolerance=30)
                self.wait_attitude(despitch=60, tolerance=30) # desroll is +/-180
                self.wait_attitude(despitch=-60, desroll=0, tolerance=30)
                self.wait_attitude(despitch=0, desroll=0, tolerance=30)

            self.progress("Flipping in pitch-back")
            gentle_positive_stick = 1700
            self.set_rc(2, gentle_positive_stick)
            self.send_cmd_do_set_mode('FLIP') # don't wait for success
            try:
                wait_for_pitch_back_flip()
            except AutoTestTimeoutException:
                raise NotAchievedException("Flip not confirmed. (If the flip did happen, our detection needs tuning.)")
            self.progress("Waiting for level")
            self.set_rc(2, neutral_stick)
            self.wait_attitude(despitch=0, desroll=0, tolerance=5)
            self.wait_mode('ALT_HOLD')
            self.wait_altitude(
                pre_flip_altitude_m - acceptable_altitude_loss_during_flip_m,
                pre_flip_altitude_m + acceptable_altitude_loss_during_flip_m,
                relative=True,
            )

        self.start_subtest("Test 3: Enter & abandon the flip using RC Aux")
        get_ready_for_flip()

        def df_log_shows_a_flip_was_abandoned() -> bool:
            """Returns true if the dataflash log shows a flip mode was abandoned."""
            dfreader = self.dfreader_for_current_onboard_log()
            err_msg_found = dfreader.recv_match(type="ERR", condition="ERR.Subsys==13 and ERR.ECode==2")
            return bool(err_msg_found)

        self.set_rc(1, neutral_stick) # forces default flip behavior
        self.set_rc(flip_rc_aux_channel, aux_low) # start low, so a switch-to-high is detected
        # Precondition: Confirm there are no flip-error messages.
        if df_log_shows_a_flip_was_abandoned():
            raise NotAchievedException("The dataflash log already has a Flip Mode error, somehow.")
        # Because the initiate-and-abandon must happen in a tight window of time, slow down sim
        self.context_push()
        self.context_set_speedup(1.0)
        # Pre-constrain tilt so that if we reach the desroll, that's definitely due to the flip
        self.set_parameter("ATC_ANGLE_MAX", 10.0)
        self.install_message_hook(flip_mode_watcher)
        self.set_rc(flip_rc_aux_channel, aux_high, timeout=None)
        self.wait_attitude(desroll=15, despitch=0, use_cached_simtime=True, timeout=30)
        self.set_rc(flip_rc_aux_channel, aux_low, timeout=None)
        self.wait_attitude(despitch=0, desroll=0, tolerance=5)
        self.wait_mode('ALT_HOLD')
        if not flip_mode_watcher.mode_was_observed:
            raise NotAchievedException("Flip mode did not occur as expected.")
        if not df_log_shows_a_flip_was_abandoned():
            raise NotAchievedException("Flip was not aborted as expected.")
        self.remove_message_hook(flip_mode_watcher)
        self.context_pop()

        self.do_RTL()
