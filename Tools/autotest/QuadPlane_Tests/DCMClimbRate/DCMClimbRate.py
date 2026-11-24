'''
QuadPlane autotest: DCMClimbRate

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class DCMClimbRate(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def DCMClimbRate(self):
        '''Test the climb rate measurement in DCM with and without GPS'''
        self.wait_ready_to_arm()

        self.change_mode('QHOVER')
        self.arm_vehicle()
        self.set_rc(3, 2000)
        self.wait_altitude(30, 50, relative=True)

        # Start Descending
        self.set_rc(3, 1000)
        self.wait_climbrate(-5, -0.5, timeout=10)

        # Switch to DCM
        self.set_parameter('AHRS_EKF_TYPE', 0)
        self.delay_sim_time(5, reason="DCM to initialise")

        # Start Climbing
        self.set_rc(3, 2000)
        self.wait_climbrate(0.5, 5, timeout=10)

        # Kill any GPSs
        self.set_parameters({
            'SIM_GPS1_ENABLE': 0,
            'SIM_GPS2_ENABLE': 0,
        })
        self.delay_sim_time(5, reason="GPS disable to take effect")

        # Start Descending
        self.set_rc(3, 1000)
        self.wait_climbrate(-5, -0.5, timeout=10)

        # reboot SITL
        self.reboot_sitl(force=True)
