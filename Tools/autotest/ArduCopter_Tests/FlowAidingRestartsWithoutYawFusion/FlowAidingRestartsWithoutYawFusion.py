'''
Copter autotest: FlowAidingRestartsWithoutYawFusion

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class FlowAidingRestartsWithoutYawFusion(TestGroup):
    @testcase(bucket="CopterTests2a")
    def FlowAidingRestartsWithoutYawFusion(self):
        '''optical flow aiding restarts after a dropout while a configured compass is not fusing'''
        # Optical flow does not learn the Z gyro bias while no yaw is being
        # fused, so that variance stays above the gyro bias learned threshold,
        # and aiding only restarts from AID_NONE once that check passes.
        self.set_parameters({
            "SIM_FLOW_ENABLE": 1,
            "FLOW_TYPE": 10,
            "SIM_GPS1_ENABLE": 0,
            "SIM_TERRAIN": 0,
            "EK3_SRC1_YAW": 1,
        })
        self.configure_EKFs_to_use_optical_flow_instead_of_GPS()
        self.set_analog_rangefinder_parameters()
        self.reboot_sitl()
        self.takeoff(10, mode='LOITER', require_absolute=False, takeoff_throttle=1800)
        # no position controller, so the flow dropout does not trigger a landing
        self.change_mode('ALT_HOLD')
        self.set_parameters({"SIM_MAG1_FAIL": 1, "SIM_MAG2_FAIL": 1, "SIM_MAG3_FAIL": 1})
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG, True, True, False, timeout=10)
        self.context_collect('STATUSTEXT')
        self.set_parameter("SIM_FLOW_ENABLE", 0)
        self.wait_statustext("EKF3 IMU0 stopped aiding", check_context=True, timeout=30)
        self.set_parameter("SIM_FLOW_ENABLE", 1)
        self.wait_statustext("EKF3 IMU0 started relative aiding", check_context=True, timeout=30)
        self.land_and_disarm()
