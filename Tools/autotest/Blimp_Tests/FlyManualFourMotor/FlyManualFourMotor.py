'''
Blimp autotest: FlyManualFourMotor

AP_FLAKE8_CLEAN
'''

from pymavlink.rotmat import Vector3

from test_discovery import TestGroup
from test_discovery import testcase


class FlyManualFourMotor(TestGroup):
    @testcase(bucket="Blimp")
    def FlyManualFourMotor(self):
        '''test manual mode on the four-motor blimp frame'''
        self.customise_SITL_commandline(
            [],
            model='blimp-motor',
            defaults_filepath=self.model_defaults_filepath('blimp-motor'),
        )

        speed_accuracy = 0.1
        heading_accuracy = 10

        self.change_mode('MANUAL')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        def stop_blimp():
            self.progress("Stopping.")
            self.set_rc(1, 1500)
            self.set_rc(2, 1500)
            self.set_rc(3, 1500)
            self.set_rc(4, 1500)
            self.wait_speed_vector(Vector3(0, 0, 0), accuracy=speed_accuracy, timeout=20)
            self.wait_yaw_speed(0, 0.2, 10)

        def stop_blimp_wait_heading():
            stop_blimp()
            self.wait_heading(0, accuracy=heading_accuracy, timeout=2)

        self.progress("Moving forward.")
        self.set_rc(2, 2000)
        self.wait_speed_vector(Vector3(1.6, 0, 0), accuracy=speed_accuracy, timeout=10)
        stop_blimp_wait_heading()

        self.progress("Moving right.")
        self.set_rc(1, 2000)
        self.wait_speed_vector(Vector3(0, 1.1, 0), accuracy=speed_accuracy, timeout=10)
        stop_blimp_wait_heading()

        self.progress("Moving backward.")
        self.set_rc(2, 1000)
        self.wait_speed_vector(Vector3(-1.6, 0, 0), accuracy=speed_accuracy, timeout=10)
        stop_blimp_wait_heading()

        self.progress("Moving left.")
        self.set_rc(1, 1000)
        self.wait_speed_vector(Vector3(0, -1.1, 0), accuracy=speed_accuracy, timeout=10)
        stop_blimp_wait_heading()

        self.progress("Moving up.")
        self.set_rc(3, 2000)
        self.wait_speed_vector(Vector3(0, 0, -1.1), accuracy=speed_accuracy, timeout=15)
        stop_blimp_wait_heading()

        self.progress("Moving down.")
        self.set_rc(3, 1000)
        self.wait_speed_vector(Vector3(0, 0, 1.1), accuracy=speed_accuracy, timeout=15)
        stop_blimp_wait_heading()

        self.progress("Yawing right.")
        self.set_rc(4, 2000)
        self.wait_yaw_speed(1.0, accuracy=speed_accuracy, timeout=10)
        stop_blimp()

        self.progress("Yawing left.")
        self.set_rc(4, 1000)
        self.wait_yaw_speed(-1.0, accuracy=speed_accuracy, timeout=10)
        stop_blimp()

        self.disarm_vehicle()
