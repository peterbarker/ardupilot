'''
Copter autotest: EK3AccelBias

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class EK3AccelBias(TestGroup):
    # Tests any EK3 accel bias is subtracted from the correct IMU data
    @testcase(bucket="CopterTests1e")
    def EK3AccelBias(self):
        '''Test EK3 Accel Bias data'''
        self.context_push()

        self.start_test("Test zero bias")
        self.delay_sim_time(2, reason="EKF to initialise")
        dfreader_tstart = self.assert_dataflash_message_field_level_at(
            "XKF2",
            "AZ",
            0.0,
            condition="XKF2.C==1",
            maintain=1,
        )

        # Add 2m/s/s bias to the second IMU
        self.set_parameters({
            'SIM_ACC2_BIAS_Z': 0.7,
        })

        self.start_subtest("Ensuring second core has bias")
        self.delay_sim_time(30, reason="EKF to learn accel bias")
        dfreader_tstart = self.assert_dataflash_message_field_level_at(
            "XKF2", "AZ", 0.7,
            condition="XKF2.C==1",
        )

        self.start_subtest("Ensuring earth frame is compensated")
        self.assert_dataflash_message_field_level_at(
            "RATE", "A", 0,
            maintain=1,
            tolerance=2,    # RATE.A is in cm/s/s
            dfreader_start_timestamp=dfreader_tstart,
        )

        # now switch the EKF to only using the second core:
        self.set_parameters({
            'SIM_ACC2_BIAS_Z': 0.0,
            "EK3_IMU_MASK": 0b10,
        })
        self.reboot_sitl()

        self.delay_sim_time(30, reason="EKF to initialise after reboot")
        dfreader_tstart = self.assert_dataflash_message_field_level_at(
            "XKF2", "AZ", 0.0,
            condition="XKF2.C==0",
        )

        # Add 2m/s/s bias to the second IMU
        self.set_parameters({
            'SIM_ACC2_BIAS_Z': 0.7,
        })

        self.start_subtest("Ensuring first core now has bias")
        self.delay_sim_time(30, reason="EKF to learn accel bias")
        dfreader_tstart = self.assert_dataflash_message_field_level_at(
            "XKF2", "AZ", 0.7,
            condition="XKF2.C==0",
        )

        self.start_subtest("Ensuring earth frame is compensated")
        self.assert_dataflash_message_field_level_at(
            "RATE", "A", 0,
            maintain=1,
            tolerance=2,  # RATE.A is in cm/s/s
            dfreader_start_timestamp=dfreader_tstart,
            verbose=True,
        )

        # revert simulated accel bias and reboot to restore EKF health
        self.context_pop()
        self.reboot_sitl()
