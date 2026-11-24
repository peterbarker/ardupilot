'''
Copter autotest: ThrottleFailsafePassthrough

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ThrottleFailsafePassthrough(TestGroup):
    @testcase(bucket="CopterTests1c")
    def ThrottleFailsafePassthrough(self):
        '''check servo passthrough on RC failsafe.  Make sure it doesn't glitch to the bad RC input value'''
        channel = 7
        trim_value = 1450
        self.set_parameters({
            'RC%u_MIN' % channel: 1000,
            'RC%u_MAX' % channel: 2000,
            'SERVO%u_MIN' % channel: 1000,
            'SERVO%u_MAX' % channel: 2000,
            'SERVO%u_TRIM' % channel: trim_value,
            'SERVO%u_FUNCTION' % channel: 146,  # scaled passthrough for channel 7
            'FS_THR_ENABLE': 1,
            'RC_FS_TIMEOUT': 10,
            'SERVO_RC_FS_MSK': 1 << (channel-1),
        })

        self.reboot_sitl()

        self.context_set_message_rate_hz('SERVO_OUTPUT_RAW', 200)

        self.set_rc(channel, 1799)
        expected_servo_output_value = 1778  # 1778 because of weird trim
        self.wait_servo_channel_value(channel, expected_servo_output_value)
        # receiver goes into failsafe with wild override values:

        def ensure_SERVO_values_never_input(mav, m):
            if m.get_type() != "SERVO_OUTPUT_RAW":
                return
            value = getattr(m, "servo%u_raw" % channel)
            if value != expected_servo_output_value and value != trim_value:
                raise NotAchievedException("Bad servo value %u received" % value)

        self.install_message_hook_context(ensure_SERVO_values_never_input)
        self.progress("Forcing receiver into failsafe")
        self.set_rc_from_map({
            3: 800,
            channel: 1300,
        })
        self.wait_servo_channel_value(channel, trim_value)
        self.delay_sim_time(10, reason="RC failsafe passthrough to settle")
