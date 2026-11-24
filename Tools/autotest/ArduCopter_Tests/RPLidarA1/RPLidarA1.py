'''
Copter autotest: RPLidarA1

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class RPLidarA1(TestGroup):
    @testcase(bucket="CopterTests2c")
    def RPLidarA1(self):
        '''test Raspberry Pi Lidar A1'''
        return  # we don't send distances when too long?
        expected_distances = {
            mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 276,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 256,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 800,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 800,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 626,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 800,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 762,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 792,
        }

        self.test_rplidar("rplidara1", expected_distances)
