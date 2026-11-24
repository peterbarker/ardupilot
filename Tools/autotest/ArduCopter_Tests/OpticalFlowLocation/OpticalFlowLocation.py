'''
Copter autotest: OpticalFlowLocation

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class OpticalFlowLocation(TestGroup):
    @testcase(bucket="CopterTests1a")
    def OpticalFlowLocation(self):
        '''test optical flow doesn't supply location'''

        self.context_push()

        self.assert_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, False, False, False, verbose=True)

        self.start_subtest("Make sure no crash if no rangefinder")
        self.set_parameter("SIM_FLOW_ENABLE", 1)
        self.set_parameter("FLOW_TYPE", 10)

        self.configure_EKFs_to_use_optical_flow_instead_of_GPS()

        self.reboot_sitl()

        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, True, True, True, verbose=True)

        self.change_mode('LOITER')
        self.wait_statustext("Need Position Estimate", timeout=300)

        self.context_pop()

        self.reboot_sitl()
