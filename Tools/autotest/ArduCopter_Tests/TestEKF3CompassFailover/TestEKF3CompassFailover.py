'''
Copter autotest: TestEKF3CompassFailover

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class TestEKF3CompassFailover(TestGroup):
    @testcase(bucket="CopterTests2b")
    def TestEKF3CompassFailover(self):
        '''Test if changed compasses goes back to primary in EKF3 on disarm'''
        # Enable only EKF3
        self.set_parameters({
            "EK3_ENABLE": 1,
        })

        self.takeoff(10, mode="LOITER")

        self.context_collect("STATUSTEXT")

        self.progress("Disabling compass")
        self.set_parameter("SIM_MAG1_FAIL", 1)

        self.wait_statustext("EKF3 IMU0 switching to compass 1", check_context=True)
        self.wait_statustext("EKF3 IMU1 switching to compass 1", check_context=True)
        self.progress("Compass switch detected")

        self.context_clear_collection("STATUSTEXT")
        self.delay_sim_time(2, reason="EKF to settle on new compass")

        self.progress("Re-enabling compass")
        self.set_parameter("SIM_MAG1_FAIL", 0)

        self.context_clear_collection("STATUSTEXT")
        self.progress("Landing and disarming")
        self.land_and_disarm()

        self.wait_statustext("EKF3 IMU0 switching to compass 0", check_context=True)
        self.wait_statustext("EKF3 IMU1 switching to compass 0", check_context=True)
        self.progress("Compass restored")

        self.progress("Landing and disarming")
        self.land_and_disarm()
