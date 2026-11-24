'''
Copter autotest: GPSFixTypes

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GPSFixTypes(TestGroup):
    @testcase(bucket="CopterTests2d")
    def GPSFixTypes(self):
        '''Test that SIM_GPS1_FIXTYPE maps correctly to GPS_RAW_INT.fix_type'''
        self.change_mode('LOITER')

        for fix_type in range(6, -1, -1):  # Only test up to RTK fixed, not PPP/Static
            self.start_subtest("Testing fix type %u" % fix_type)
            self.set_parameter("SIM_GPS1_FIXTYPE", fix_type)
            self.wait_message_field_values('GPS_RAW_INT', {
                "fix_type": fix_type,
            }, timeout=10)
            self.set_parameter("SIM_GPS1_FIXTYPE", 6)
            self.wait_ready_to_arm()
