'''
Rover autotest: OpticalFlow

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class OpticalFlow(TestGroup):
    @testcase(bucket="Rover")
    def OpticalFlow(self):
        '''lightly test OpticalFlow'''
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, False, False, False, verbose=True)

        self.context_push()
        self.set_parameter("SIM_FLOW_ENABLE", 1)
        self.set_parameter("FLOW_TYPE", 10)

        self.reboot_sitl()
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW, True, True, True, verbose=True)

        self.context_pop()
        self.reboot_sitl()
