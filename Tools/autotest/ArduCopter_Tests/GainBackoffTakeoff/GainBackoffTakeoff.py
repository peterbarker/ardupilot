'''
Copter autotest: GainBackoffTakeoff

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

import vehicle_test_suite

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GainBackoffTakeoff(TestGroup):
    @testcase(bucket="CopterTests2c")
    def GainBackoffTakeoff(self):
        '''test gain backoff on takeoff'''
        self.context_push()
        self.progress("Test gains are fully backed off")
        self.set_parameters({
            "ATC_LAND_R_MULT": 0.0,
            "ATC_LAND_P_MULT": 0.0,
            "ATC_LAND_Y_MULT": 0.0,
            "GCS_PID_MASK" : 7,
            "LOG_BITMASK": 180222,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.change_mode('ALT_HOLD')

        class ValidatePDZero(vehicle_test_suite.TestSuite.MessageHook):
            '''asserts correct values in PID_TUNING'''

            def __init__(self, suite, axis):
                super(ValidatePDZero, self).__init__(suite)
                self.pid_tuning_count = 0
                self.p_sum = 0
                self.d_sum = 0
                self.i_sum = 0
                self.axis = axis

            def hook_removed(self):
                if self.pid_tuning_count == 0:
                    raise NotAchievedException("Did not get PID_TUNING")
                if self.i_sum == 0:
                    raise ValueError("I sum is zero")
                print(f"ValidatePDZero: PID_TUNING count: {self.pid_tuning_count}")

            def process(self, mav, m):
                if m.get_type() != 'PID_TUNING' or m.axis != self.axis:
                    return
                self.pid_tuning_count += 1
                self.p_sum += m.P
                self.d_sum += m.D
                self.i_sum += m.I
                if self.p_sum > 0:
                    raise ValueError("P sum is not zero")
                if self.d_sum > 0:
                    raise ValueError("D sum is not zero")

        self.progress("Check that PD values are zero")
        self.install_message_hook_context(ValidatePDZero(self, mavutil.mavlink.PID_TUNING_ROLL))
        self.install_message_hook_context(ValidatePDZero(self, mavutil.mavlink.PID_TUNING_PITCH))
        self.install_message_hook_context(ValidatePDZero(self, mavutil.mavlink.PID_TUNING_YAW))
        # until the context pop happens, all received PID_TUNINGS will be verified as good
        self.arm_vehicle()
        self.set_rc(3, 1500)
        self.delay_sim_time(2, reason="motors to spool up")
        self.set_rc(2, 1250)
        self.delay_sim_time(5, reason="PID data to accumulate")
        self.assert_receive_message('PID_TUNING', timeout=5)
        self.set_rc_default()
        self.zero_throttle()
        self.disarm_vehicle()
        self.context_pop()

        self.context_push()
        self.progress("Test gains are not backed off")
        self.set_parameters({
            "ATC_LAND_R_MULT": 1.0,
            "ATC_LAND_P_MULT": 1.0,
            "ATC_LAND_Y_MULT": 1.0,
            "GCS_PID_MASK" : 7,
            "LOG_BITMASK": 180222,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.change_mode('ALT_HOLD')

        class ValidatePDNonZero(vehicle_test_suite.TestSuite.MessageHook):
            '''asserts correct values in PID_TUNING'''

            def __init__(self, suite, axis):
                super(ValidatePDNonZero, self).__init__(suite)
                self.pid_tuning_count = 0
                self.p_sum = 0
                self.d_sum = 0
                self.i_sum = 0
                self.axis = axis

            def hook_removed(self):
                if self.pid_tuning_count == 0:
                    raise NotAchievedException("Did not get PID_TUNING")
                if self.p_sum == 0:
                    raise ValueError("P sum is zero")
                if self.i_sum == 0:
                    raise ValueError("I sum is zero")
                print(f"ValidatePDNonZero: PID_TUNING count: {self.pid_tuning_count}")

            def process(self, mav, m):
                if m.get_type() != 'PID_TUNING' or m.axis != self.axis:
                    return
                self.pid_tuning_count += 1
                self.p_sum += m.P
                self.d_sum += m.D
                self.i_sum += m.I

        self.progress("Check that PD values are non-zero")
        self.install_message_hook_context(ValidatePDNonZero(self, mavutil.mavlink.PID_TUNING_ROLL))
        self.install_message_hook_context(ValidatePDNonZero(self, mavutil.mavlink.PID_TUNING_PITCH))
        self.install_message_hook_context(ValidatePDNonZero(self, mavutil.mavlink.PID_TUNING_YAW))
        # until the context pop happens, all received PID_TUNINGS will be verified as good
        self.arm_vehicle()
        self.set_rc(3, 1500)
        self.delay_sim_time(2, reason="motors to spool up")
        self.set_rc(2, 1250)
        self.delay_sim_time(5, reason="PID data to accumulate")
        self.assert_receive_message('PID_TUNING', timeout=5)
        self.set_rc_default()
        self.zero_throttle()
        self.disarm_vehicle()

        self.context_pop()
        self.reboot_sitl()
