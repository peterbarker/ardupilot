'''
Copter autotest: BatteryInternalUseOnly

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryInternalUseOnly(TestGroup):
    @testcase(bucket="CopterTests1a")
    def BatteryInternalUseOnly(self):
        '''batteries marked as internal use only should not appear over mavlink'''
        self.set_parameters({
            "BATT_MONITOR": 4,  # 4 is analog volt+curr
            "BATT2_MONITOR": 4,
        })
        self.reboot_sitl()
        self.wait_message_field_values('BATTERY_STATUS', {
            "id": 0,
        })
        self.wait_message_field_values('BATTERY_STATUS', {
            "id": 1,
        })
        self.progress("Making battery private")
        self.set_parameters({
            "BATT_OPTIONS": 256,
        })
        self.wait_message_field_values('BATTERY_STATUS', {
            "id": 1,
        })
        for i in range(10):
            self.assert_received_message_field_values('BATTERY_STATUS', {
                "id": 1
            })
