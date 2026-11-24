'''
Copter autotest: BaseLoggingRates

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BaseLoggingRates(TestGroup):
    @testcase(bucket="CopterTests2b")
    def BaseLoggingRates(self):
        '''ensure messages come out at specific rates'''
        path = self.generate_rate_sample_log()
        self.check_dflog_message_rates(path, {
            "ATT": 10,
            "IMU": 25,
        })
