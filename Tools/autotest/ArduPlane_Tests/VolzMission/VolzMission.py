'''
Plane autotest: VolzMission

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class VolzMission(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def VolzMission(self):
        '''test Volz serially-connected servos in a mission'''
        volz_motor_mask = ((1 << 0) | (1 << 1) | (1 << 3) | (1 << 8) | (1 << 9) | (1 << 11))
        self.set_parameters({
            'SERIAL5_PROTOCOL': 14,
            'SERVO_VOLZ_MASK': volz_motor_mask,
            'RTL_AUTOLAND': 2,

            'SIM_VOLZ_ENA': 1,
            'SIM_VOLZ_MASK': volz_motor_mask,
        })
        # defaults file not working?
        self.set_parameters({
            "SERVO2_REVERSED":  0,  # elevator

            "SERVO9_FUNCTION": 4,

            "SERVO10_FUNCTION": 19,  # elevator

            "SERVO12_FUNCTION": 21,  # rudder
            "SERVO12_REVERSED":  1,  # rudder

        })
        self.customise_SITL_commandline([
            "--serial5=sim:volz",
        ], model="plane-redundant",
                                        )
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.takeoff()
        self.fly_home_land_and_disarm()
