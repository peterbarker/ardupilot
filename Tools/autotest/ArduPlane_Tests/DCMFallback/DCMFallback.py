'''
Plane autotest: DCMFallback

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class DCMFallback(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def DCMFallback(self):
        '''Really annoy the EKF and force fallback'''
        self.reboot_sitl()
        self.delay_sim_time(30, reason="SITL to initialise after reboot")

        self.takeoff(50)
        self.change_mode('CIRCLE')
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "EK3_POS_I_GATE": 0,
            "SIM_GPS1_HZ": 1,
            "SIM_GPS1_LAG_MS": 1000,
        })
        self.wait_statustext("DCM Active", check_context=True, timeout=60)
        self.wait_statustext("EKF3 Active", check_context=True)
        self.wait_statustext("DCM Active", check_context=True)
        self.wait_statustext("EKF3 Active", check_context=True)
        self.wait_statustext("DCM Active", check_context=True)
        self.wait_statustext("EKF3 Active", check_context=True)
        self.context_stop_collecting('STATUSTEXT')

        self.fly_home_land_and_disarm()
        self.context_pop()
        self.reboot_sitl()
