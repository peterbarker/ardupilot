'''
Copter autotest: EK3_AccelBiasZeroVelOptFlow

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class EK3_AccelBiasZeroVelOptFlow(TestGroup):
    @testcase(bucket="CopterTests1e")
    def EK3_AccelBiasZeroVelOptFlow(self):
        '''Test EKF3 zero velocity fusion learns bias with optical flow config'''
        # When optical flow is configured (AID_RELATIVE) but the vehicle is
        # stationary on the ground, optical flow provides no velocity data.
        # Without zero velocity fusion, there are no velocity observations and
        # accel biases cannot converge. This test verifies that synthetic zero
        # velocity fusion fills the gap, allowing Z-axis bias learning even
        # when the only velocity source (optical flow) has no data.
        self.context_push()

        self.set_parameters({
            "LOG_FILE_DSRMROT": 1,
            "SIM_FLOW_ENABLE": 1,
            "FLOW_TYPE": 10,
        })
        self.set_analog_rangefinder_parameters()
        self.configure_EKFs_to_use_optical_flow_instead_of_GPS()

        self.reboot_sitl()
        self.wait_ready_to_arm(timeout=120, require_absolute=False)

        # Inject Z-bias on IMU2 and wait for EKF to learn it via zero velocity
        # fusion. In AID_RELATIVE with no flow data, only the synthetic zero
        # velocity provides the observation needed to make Z-bias converge.
        self.start_subtest("Verify Z-bias learned via zero velocity fusion with optical flow config")
        self.set_parameters({
            'SIM_ACC2_BIAS_Z': 0.7,
        })
        self.delay_sim_time(30, reason="EKF to learn accel bias via zero velocity fusion")
        self.assert_dataflash_message_field_level_at(
            "XKF2", "AZ", 0.7,
            condition="XKF2.C==1",
            maintain=1,
        )

        self.context_pop()
        self.reboot_sitl()
