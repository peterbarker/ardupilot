'''
Helicopter autotest: TurbineStart

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TurbineStart(TestGroup):
    @testcase(bucket="Helicopter")
    def TurbineStart(self, timeout=200):
        """Check Turbine Start Feature"""
        RAMP_TIME = 4
        # set option for Turbine Start
        self.set_parameter("RC6_OPTION", 161)
        self.set_parameter("H_RSC_RAMP_TIME", RAMP_TIME)
        self.set_parameter("H_RSC_SETPOINT", 66)
        self.set_parameter("DISARM_DELAY", 0)
        self.set_rc_from_map({3: 1000, 8: 1000})

        # check that turbine start doesn't activate while disarmed
        self.progress("Checking Turbine Start doesn't activate while disarmed")
        self.set_rc(6, 2000)
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 2:
            servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
            if servo.servo8_raw > 1050:
                raise NotAchievedException("Turbine Start activated while disarmed")
        self.set_rc(6, 1000)

        # check that turbine start doesn't activate armed with interlock enabled
        self.progress("Checking Turbine Start doesn't activate while armed with interlock enabled")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc_from_map({8: 2000, 6: 2000})
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 5:
            servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
            if servo.servo8_raw > 1660:
                raise NotAchievedException("Turbine Start activated with interlock enabled")

        self.set_rc_from_map({8: 1000, 6: 1000})
        self.disarm_vehicle()

        # check that turbine start activates as designed (armed with interlock disabled)
        self.progress("Checking Turbine Start activates as designed (armed with interlock disabled)")
        self.delay_sim_time(2, reason="vehicle to fully disarm")
        self.arm_vehicle()

        self.set_rc(6, 2000)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > 5:
                raise AutoTestTimeoutException("Turbine Start did not activate")
            servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
            if servo.servo8_raw > 1800:
                break

        self.wait_servo_channel_value(8, 1000, timeout=3)
        self.set_rc(6, 1000)

        # check that turbine start will not reactivate after interlock enabled
        self.progress("Checking Turbine Start doesn't activate once interlock is enabled after start)")
        self.set_rc_from_map({8: 2000, 6: 2000})
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 5:
            servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
            if servo.servo8_raw > 1660:
                raise NotAchievedException("Turbine Start activated with interlock enabled")
        self.set_rc_from_map({6: 1000, 8: 1000})
        self.disarm_vehicle()
