'''
Sub autotest: SHT3X

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SHT3X(TestGroup):
    @testcase(bucket="Sub")
    def SHT3X(self):
        '''test for the SHT3X temperature/hygro driver'''
        self.set_parameters({
            'TEMP1_TYPE': 8,
            'TEMP1_ADDR': 0x44,
            'TEMP_LOG': 1,
        })
        self.reboot_sitl()
        self.context_push()
        self.set_parameter('LOG_DISARMED', 1)
        self.delay_sim_time(10, reason="temperature data to be logged")
        self.context_pop()

        dfreader = self.dfreader_for_current_onboard_log()
        temp_min = 35
        temp_max = 45
        while True:
            m = dfreader.recv_match(type='TEMP')
            if m is None:
                break
            self.progress(m)
            if temp_min < m.Temp < temp_max:
                # success!
                break
        if m is None:
            raise NotAchievedException(
                f"Did not get good TEMP message (want {temp_min} < Temp < {temp_max})")
