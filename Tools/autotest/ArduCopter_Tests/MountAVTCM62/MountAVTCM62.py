'''
Copter autotest: MountAVTCM62

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MountAVTCM62(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MountAVTCM62(self):
        '''test MAVLink (Gimbal Protocol v2) gimbal using SIM_AVT_CM62 simulator'''
        self.set_parameters({
            "MNT1_TYPE": 6,         # MAVLink
            "CAM1_TYPE": 4,         # Mount
            "SERIAL5_PROTOCOL": 2,  # MAVLink2
            "RC6_OPTION": 213,      # MOUNT1_PITCH
        })
        self.customise_SITL_commandline(["--serial5=sim:avt_cm62_gimbal:"])

        self.progress("Capture on an unconfigured camera instance must be denied")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
            p1=2,  # camera instance 2 is not configured
            p2=0,  # interval
            p3=1,  # total images
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )
