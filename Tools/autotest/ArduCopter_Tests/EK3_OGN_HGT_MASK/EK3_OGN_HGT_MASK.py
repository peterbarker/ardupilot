'''
Copter autotest: EK3_OGN_HGT_MASK

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class EK3_OGN_HGT_MASK(TestGroup):
    @testcase(bucket="CopterTests2d")
    def EK3_OGN_HGT_MASK(self):
        '''test baraometer-alt-compensation based on long-term GPS readings'''
        self.context_push()
        self.set_parameters({
            'EK3_OGN_HGT_MASK': 1,  # compensate baro drift using GPS
        })
        self.reboot_sitl()

        expected_alt = 10

        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        current_alt = self.get_altitude()

        expected_alt_abs = current_alt + expected_alt

        self.takeoff(expected_alt, mode='GUIDED')
        self.delay_sim_time(5, reason="EKF to stabilise at target altitude")

        self.set_parameter("SIM_BARO_DRIFT", 0.01)  # 1cm/second

        def check_altitude(mav, m):
            m_type = m.get_type()
            epsilon = 10  # in metres
            if m_type == 'GPS_RAW_INT':
                got_gps_alt = m.alt * 0.001
                if abs(expected_alt_abs - got_gps_alt) > epsilon:
                    raise NotAchievedException(f"Bad GPS altitude (got={got_gps_alt} want={expected_alt_abs})")
            elif m_type == 'GLOBAL_POSITION_INT':
                got_canonical_alt = m.relative_alt * 0.001
                if abs(expected_alt - got_canonical_alt) > epsilon:
                    raise NotAchievedException(f"Bad canonical altitude (got={got_canonical_alt} want={expected_alt})")

        self.install_message_hook_context(check_altitude)

        self.delay_sim_time(1500, reason="baro drift to accumulate while checking altitude hold")

        self.disarm_vehicle(force=True)

        self.context_pop()

        self.reboot_sitl(force=True)
