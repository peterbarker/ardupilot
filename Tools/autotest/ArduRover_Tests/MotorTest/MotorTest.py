'''
Rover autotest: MotorTest

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MotorTest(TestGroup):
    # self.GCSRally,  # enable me in 2026, needs MAVProxy fixes
    @testcase(bucket="Rover")
    def MotorTest(self):
        '''Motor Test triggered via mavlink'''
        magic_throttle_value = 1812
        self.wait_ready_to_arm()
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            p1=1, # motor instance
            p2=mavutil.mavlink.MOTOR_TEST_THROTTLE_PWM, # throttle type
            p3=magic_throttle_value, # throttle
            p4=5, # timeout
            p5=1, # motor count
            p6=0, # test order (see MOTOR_TEST_ORDER)
        )
        self.wait_armed()
        self.progress("Waiting for magic throttle value")
        self.wait_servo_channel_value(3, magic_throttle_value)
        self.wait_servo_channel_value(3, self.get_parameter("RC3_TRIM", 5), timeout=10)
        self.wait_disarmed()
