'''
Rover autotest: MAVProxy_SetModeUsingMode

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MAVProxy_SetModeUsingMode(TestGroup):
    @testcase(bucket="Rover")
    def MAVProxy_SetModeUsingMode(self):
        '''Set modes via mavproxy mode command'''
        fnoo = [(1, 'ACRO'),
                (3, 'STEERING'),
                (4, 'HOLD'),
                ]
        mavproxy = self.start_mavproxy()
        for (num, expected) in fnoo:
            mavproxy.send('mode manual\n')
            self.wait_mode("MANUAL")
            mavproxy.send('mode %u\n' % num)
            self.wait_mode(expected)
            mavproxy.send('mode manual\n')
            self.wait_mode("MANUAL")
            mavproxy.send('mode %s\n' % expected)
            self.wait_mode(expected)
        self.stop_mavproxy(mavproxy)
