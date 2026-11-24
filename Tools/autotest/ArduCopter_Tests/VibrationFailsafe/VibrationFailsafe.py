'''
Copter autotest: VibrationFailsafe

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class VibrationFailsafe(TestGroup):
    @testcase(bucket="CopterTests1c")
    def VibrationFailsafe(self):
        '''Test Vibration Failsafe'''
        self.context_push()

        # takeoff in Loiter to 20m
        self.takeoff(20, mode="LOITER")

        # simulate accel bias caused by high vibration
        self.set_parameters({
            'SIM_ACC1_BIAS_Z': 2,
            'SIM_ACC2_BIAS_Z': 2,
            'SIM_ACC3_BIAS_Z': 2,
        })

        # wait for Vibration compensation warning and change to LAND mode
        self.wait_statustext("Vibration compensation ON", timeout=30)
        self.change_mode("LAND")

        # check vehicle descends to 2m or less within 40 seconds
        self.wait_altitude(-5, 2, timeout=50, relative=True)

        # force disarm of vehicle (it will likely not automatically disarm)
        self.disarm_vehicle(force=True)

        # revert simulated accel bias and reboot to restore EKF health
        self.context_pop()
        self.reboot_sitl()
