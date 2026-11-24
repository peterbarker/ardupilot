'''
Rover autotest: MAVProxy_DO_SET_MODE

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MAVProxy_DO_SET_MODE(TestGroup):
    @testcase(bucket="Rover")
    def MAVProxy_DO_SET_MODE(self):
        '''Set mode using MAVProxy commandline DO_SET_MODE'''
        mavproxy = self.start_mavproxy()
        self.mavproxy_do_set_mode_via_command_long(mavproxy, "HOLD")
        self.mavproxy_do_set_mode_via_command_long(mavproxy, "MANUAL")
        self.stop_mavproxy(mavproxy)
