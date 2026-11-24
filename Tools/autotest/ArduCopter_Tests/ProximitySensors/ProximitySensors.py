'''
Copter autotest: ProximitySensors

AP_FLAKE8_CLEAN
'''

import copy

from pymavlink import mavutil

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ProximitySensors(TestGroup):
    @testcase(bucket="CopterTests1c")
    def ProximitySensors(self):
        '''ensure proximity sensors return appropriate data'''

        self.set_parameters({
            "SERIAL5_PROTOCOL": 11,
            "OA_DB_OUTPUT": 3,
            "OA_TYPE": 2,
        })
        sensors = [  # tuples of name, prx_type
            ('ld06', 16, {
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 275,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 256,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 1130,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 1200,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 625,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 967,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 760,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 765,
            }),
            ('sf45b', 8, {
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 270,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 258,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 1146,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 632,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 629,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 972,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 774,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 774,
            }),
            ('rplidara2', 5, {
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 277,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 256,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 1130,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 1288,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 626,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 970,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 762,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 790,
            }),
            ('rplidars2', 5, {
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 274,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 256,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 1129,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 677,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 625,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 967,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 760,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 765,
            }),
            ('terarangertower', 3, {
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 450,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 282,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 450,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 450,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 450,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 450,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 450,
                mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 450,
            }),
        ]

        # the following is a "magic" location SITL understands which
        # has some posts near it:
        home_string = "%s,%s,%s,%s" % (51.8752066, 14.6487840, 54.15, 0)
        for (name, prx_type, expected_distances) in sensors:
            self.start_subtest("Testing %s" % name)
            self.set_parameter("PRX1_TYPE", prx_type)
            self.customise_SITL_commandline([
                "--serial5=sim:%s:" % name,
                "--home", home_string,
            ])
            self.wait_ready_to_arm()
            expected_distances_copy = copy.copy(expected_distances)
            tstart = self.get_sim_time()
            failed = False
            wants = []
            gots = []
            epsilon = 20
            while True:
                if self.get_sim_time_cached() - tstart > 30:
                    raise AutoTestTimeoutException("Failed to get distances")
                if len(expected_distances_copy.keys()) == 0:
                    break
                m = self.assert_receive_message("DISTANCE_SENSOR")
                if m.orientation not in expected_distances_copy:
                    continue
                got = m.current_distance
                want = expected_distances_copy[m.orientation]
                wants.append(want)
                gots.append(got)
                if abs(want - got) > epsilon:
                    failed = True
                del expected_distances_copy[m.orientation]
            if failed:
                raise NotAchievedException(
                    "Distance too great (%s) (want=%s != got=%s)" %
                    (name, wants, gots))
