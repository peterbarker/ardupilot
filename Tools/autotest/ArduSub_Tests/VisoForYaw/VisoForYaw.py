'''
Sub autotest: VisoForYaw

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase

from ardusub import Joystick


class VisoForYaw(TestGroup):
    @testcase(bucket="Sub")
    def VisoForYaw(self):
        ''' Test propagation of yaw from VisualOdom to EKF'''
        yaw_offset_deg = 50
        # Configure SITL to use Vicon
        self.customise_SITL_commandline(["--serial5=sim:vicon:"])

        # Configure EKF to use Vicon for yaw
        self.set_parameters({
            "VISO_TYPE": 1, # Vicon
            "SERIAL5_PROTOCOL": 1,
            "SIM_VICON_TMASK": 1,  # Send POSITION_ESTIMATE
            "EK3_SRC1_YAW": 6, # Tell EKF to use Vicon for yaw
        })
        self.reboot_sitl()

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('STABILIZE')
        # Look east
        self.set_rc(Joystick.Yaw, 1600)
        self.wait_heading(90)
        self.set_rc(Joystick.Yaw, 1500)
        self.wait_yaw_speed(0, minimum_duration=1)

        prev_yaw_deg = self.get_heading()
        # Change to manual to prevent the vehicle from trying to correct yaw
        self.change_mode('MANUAL')
        self.set_parameters({
            'EK3_GYRO_P_NSE': 1.0, # Increased gyro noise makes EKF more sensitive to Vicon yaw
        })
        self.set_parameter('SIM_VICON_YAWERR', yaw_offset_deg)
        self.wait_heading(prev_yaw_deg + yaw_offset_deg, timeout=120)
        self.set_parameter('SIM_VICON_YAWERR', 0)
        self.wait_heading(prev_yaw_deg, timeout=120)

        self.disarm_vehicle()
        self.progress("VisualOdom for Yaw OK")
