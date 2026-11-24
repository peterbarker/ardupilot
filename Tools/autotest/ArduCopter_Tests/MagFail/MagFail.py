'''
Copter autotest: MagFail

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MagFail(TestGroup):
    @testcase(bucket="CopterTests1d")
    def MagFail(self):
        '''test failover of compass in EKF'''
        # we want both EK2 and EK3
        self.set_parameters({
            "EK2_ENABLE": 1,
            "EK3_ENABLE": 1,
        })

        self.takeoff(10, mode="LOITER")

        self.change_mode('CIRCLE')

        self.delay_sim_time(20, reason="copter to circle and build attitude history")

        self.context_collect("STATUSTEXT")

        self.progress("Failing first compass")
        self.set_parameter("SIM_MAG1_FAIL", 1)

        # we want for the message twice, one for EK2 and again for EK3
        self.wait_statustext("EKF2 IMU0 switching to compass 1", check_context=True)
        self.wait_statustext("EKF3 IMU0 switching to compass 1", check_context=True)
        self.progress("compass switch 1 OK")

        self.delay_sim_time(2, reason="EKF to settle on new compass")

        self.context_clear_collection("STATUSTEXT")

        self.progress("Failing 2nd compass")
        self.set_parameter("SIM_MAG2_FAIL", 1)

        self.wait_statustext("EKF2 IMU0 switching to compass 2", check_context=True)
        self.wait_statustext("EKF3 IMU0 switching to compass 2", check_context=True)

        self.progress("compass switch 2 OK")

        self.delay_sim_time(2, reason="EKF to settle on new compass")

        self.context_clear_collection("STATUSTEXT")

        self.progress("Failing 3rd compass")
        self.set_parameter("SIM_MAG3_FAIL", 1)
        self.delay_sim_time(2, reason="EKF to detect compass failure")
        self.set_parameter("SIM_MAG1_FAIL", 0)

        self.wait_statustext("EKF2 IMU0 switching to compass 0", check_context=True)
        self.wait_statustext("EKF3 IMU0 switching to compass 0", check_context=True)
        self.progress("compass switch 0 OK")

        self.do_RTL()
