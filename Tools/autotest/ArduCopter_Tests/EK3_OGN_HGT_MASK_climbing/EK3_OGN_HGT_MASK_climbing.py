'''
Copter autotest: EK3_OGN_HGT_MASK_climbing

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class EK3_OGN_HGT_MASK_climbing(TestGroup):
    @testcase(bucket="CopterTests2c")
    def EK3_OGN_HGT_MASK_climbing(self):
        '''check combination of height bits doesn't cause climb'''
        self.context_push()
        self.set_parameters({
            'EK3_OGN_HGT_MASK': 5,
        })
        self.reboot_sitl()

        expected_alt = 10

        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        current_alt = self.get_altitude()

        expected_alt_abs = current_alt + expected_alt

        self.takeoff(expected_alt, mode='GUIDED')
        self.delay_sim_time(5, reason="EKF to stabilise at target altitude")

        def check_altitude(mav, m):
            m_type = m.get_type()
            epsilon = 10  # in metres
            if m_type == 'GPS_RAW_INT':
                got_gps_alt = m.alt * 0.001
                if abs(expected_alt_abs - got_gps_alt) > epsilon:
                    raise NotAchievedException(f"Bad GPS altitude (got={got_gps_alt} want={expected_alt_abs})")
            elif m_type == 'GLOBAL_POSITION_INT':
                if abs(expected_alt - m.relative_alt * 0.001) > epsilon:
                    raise NotAchievedException("Bad canonical altitude")

        self.install_message_hook_context(check_altitude)

        self.delay_sim_time(1500, reason="OGN height mask to cause altitude change")

        self.disarm_vehicle(force=True)

        self.context_pop()
        self.reboot_sitl(force=True)
