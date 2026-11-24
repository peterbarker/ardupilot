'''
Plane autotest: RCDisableAirspeedUse

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class RCDisableAirspeedUse(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def RCDisableAirspeedUse(self):
        '''Test RC DisableAirspeedUse option'''
        self.set_parameter("RC9_OPTION", 106)
        self.delay_sim_time(5, reason="RC option to take effect")
        self.set_rc(9, 1000)
        self.wait_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
            True,
            True,
            True)
        self.set_rc(9, 2000)
        self.wait_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
            True,
            False,
            True)
        self.set_rc(9, 1000)
        self.wait_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
            True,
            True,
            True)
