'''
Copter autotest: ATTITUDE_FAST

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ATTITUDE_FAST(TestGroup):
    @testcase(bucket="CopterTests1c")
    def ATTITUDE_FAST(self):
        '''ensure that when ATTITUDE_FAST is set we get many messages'''

        # enable fast logging (bit 1)
        # disable PID logging (bit 12) to avoid slowdowns in attitude logging in SITL
        log_bitmask_old = int(self.get_parameter('LOG_BITMASK'))
        log_bitmask_new = (log_bitmask_old | (1 << 0)) & ~(1 << 12)

        path = self.generate_rate_sample_log(log_bitmask=log_bitmask_new)

        self.check_dflog_message_rates(path, {
            'ANG': 400,
        })
