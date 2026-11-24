'''
Rover autotest: RCOverrideEnableChannel

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RCOverrideEnableChannel(TestGroup):
    @testcase(bucket="Rover", speedup=10)
    def RCOverrideEnableChannel(self):
        '''Test RC Override Enable channel not overridden by GCS (issue #33161)'''
        self.set_parameters({
            "MAV_GCS_SYSID": self.mav.source_system,
            "RC10_OPTION": 46,  # RC Override Enable
        })

        self.change_mode('MANUAL')

        # TX: steering centred, ch10 HIGH (enable GCS overrides)
        steering_tx = 1500
        self.set_rc_from_map({
            1: steering_tx,
            3: 1500,
            10: 2000,
        })

        # GCS joystick: full-right steering on ch1, but ch10=LOW which
        # contradicts the TX and should trigger the intermittent override bug
        steering_override = 1000
        override_ch10 = 1000

        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 15:
            self.mav.mav.rc_channels_override_send(
                1,                 # target_system
                1,                 # target_component
                steering_override, # chan1_raw (full left)
                65535,             # chan2_raw (ignore)
                65535,             # chan3_raw (ignore)
                65535,             # chan4_raw
                65535,             # chan5_raw
                65535,             # chan6_raw
                65535,             # chan7_raw
                65535,             # chan8_raw
                chan10_raw=override_ch10,  # ch10 LOW — contradicts TX HIGH
            )

            m = self.assert_receive_message('RC_CHANNELS')
            if m.chan1_raw == steering_tx:
                raise NotAchievedException(
                    "chan1 dropped to TX value (issue #33161 reproduced): "
                    "chan1_raw=%u chan10_raw=%u" %
                    (m.chan1_raw, m.chan10_raw)
                )

        # switch ch10 LOW to disable overrides; verify GCS overrides are now blocked
        self.set_rc(10, 1000)
        self.delay_sim_time(0.5, "allow debounce to complete")

        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 15:
            self.mav.mav.rc_channels_override_send(
                1,                 # target_system
                1,                 # target_component
                steering_override, # chan1_raw (should now be blocked)
                65535,             # chan2_raw (ignore)
                65535,             # chan3_raw (ignore)
                65535,             # chan4_raw
                65535,             # chan5_raw
                65535,             # chan6_raw
                65535,             # chan7_raw
                65535,             # chan8_raw
                chan10_raw=2000,   # try to re-enable via override — must be ignored
            )

            m = self.assert_receive_message('RC_CHANNELS')
            if m.chan1_raw == steering_override:
                raise NotAchievedException(
                    "chan1 accepted override when switch disabled: "
                    "chan1_raw=%u chan10_raw=%u" %
                    (m.chan1_raw, m.chan10_raw)
                )
