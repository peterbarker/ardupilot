'''
Tracker autotest: BaseMessageSet

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BaseMessageSet(TestGroup):
    @testcase(bucket="Tracker")
    def BaseMessageSet(self):
        '''ensure we're getting messages we expect'''
        self.set_parameter('BATT_MONITOR', 4)
        self.reboot_sitl()
        for msg in 'BATTERY_STATUS', :
            self.assert_receive_message(msg)
