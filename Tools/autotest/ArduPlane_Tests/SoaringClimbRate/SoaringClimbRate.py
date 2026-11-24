'''
Plane autotest: SoaringClimbRate

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase

from arduplane import AutoTestPlane


class SoaringClimbRate(TestGroup):
    @testcase(bucket="PlaneTests1b", disabled="very bad sink rate")
    def SoaringClimbRate(self):
        '''test displayed climb rate when soaring'''
        self.set_parameters({
            'RC16_OPTION': 88,  # soaring enable
            'SOAR_ENABLE': 1,
        })
        self.set_rc(16, 1000)  # disable soaring
        self.reboot_sitl()
        self.set_message_rate_hz('SIM_STATE', 10)
        self.install_message_hook_context(AutoTestPlane.ValidateVFRHudClimbAgainstSimState(self))
        self.takeoff(20)
        self.change_mode('FBWB')
        self.set_rc(2, 1000)  # full climb
        self.delay_sim_time(10, reason="climb rate to establish")
        self.set_rc(16, 2000)  # enable soaring
        self.delay_sim_time(10, reason="soaring climb rate data collection")

        self.set_rc(2, 1500)
        self.fly_home_land_and_disarm()
