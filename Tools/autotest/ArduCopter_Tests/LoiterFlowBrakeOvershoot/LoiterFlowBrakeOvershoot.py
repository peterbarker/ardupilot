'''
Copter autotest: LoiterFlowBrakeOvershoot

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class LoiterFlowBrakeOvershoot(TestGroup):
    @testcase(bucket="CopterTests2b")
    def LoiterFlowBrakeOvershoot(self):
        '''Forward-jab overshoot in optical-flow Loiter at low height'''
        # Optical flow, no GPS, low height: the EKF flow speed limit is small,
        # so AC_Loiter's drag term (which shapes the desired velocity) is large.
        # The drag is removed from the desired velocity but NOT from the
        # acceleration feed-forward passed to the position controller, so a
        # forward stick drives the vehicle past its desired-velocity trajectory;
        # position overshoots the target and the loop yanks it back (backward
        # pitch on release). High LOIT_ANG_MAX inflates the mismatch. This test
        # flies a deterministic forward jab + release for log comparison.
        self.set_parameters({
            "AHRS_EKF_TYPE": 3,
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 0,
            "SIM_FLOW_ENABLE": 1,
            "FLOW_TYPE": 10,
            "SIM_GPS1_ENABLE": 0,
            "SIM_TERRAIN": 0,
            "LOIT_ANG_MAX": 30,
            "LOIT_SPEED_MS": 5,
            "LOIT_ACC_MAX_M": 2,
        })
        self.configure_EKFs_to_use_optical_flow_instead_of_GPS()
        self.set_analog_rangefinder_parameters()
        self.reboot_sitl()

        self.wait_ready_to_arm(require_absolute=False, timeout=120)
        # flow is not healthy stationary, so climb in ALT_HOLD to a low hover
        self.takeoff(altitude_min=2, mode='ALT_HOLD', require_absolute=False, takeoff_throttle=1700)
        self.change_mode('LOITER')
        self.delay_sim_time(5, "let altitude settle")

        # deterministic forward jab then release (the RCIN.C2 dip marks it in the log)
        self.set_rc(2, 1100)
        self.delay_sim_time(3, "let log data accumulate")
        self.set_rc(2, 1500)
        self.delay_sim_time(8, "let vehicle settle and log data accumulate")

        self.disarm_vehicle(force=True)
