'''
Copter autotest: RPLidarA2

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class RPLidarA2(TestGroup):
    @testcase(bucket="CopterTests2a")
    def RPLidarA2(self):
        '''test Raspberry Pi Lidar A2'''
        expected_distances = {
            mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 276,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 256,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 1130,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 1286,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 626,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 971,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 762,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 792,
        }

        self.test_rplidar("rplidara2", expected_distances)
