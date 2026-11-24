'''
Rover autotest: MAVProxyParam

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MAVProxyParam(TestGroup):
    @testcase(bucket="Rover")
    def MAVProxyParam(self):
        '''Test MAVProxy parameter handling'''
        mavproxy = self.start_mavproxy()
        mavproxy.send("param fetch\n")
        mavproxy.expect("Received [0-9]+ parameters")
        self.stop_mavproxy(mavproxy)
