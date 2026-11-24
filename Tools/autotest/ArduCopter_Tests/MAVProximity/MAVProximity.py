'''
Copter autotest: MAVProximity

AP_FLAKE8_CLEAN
'''

import copy

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAVProximity(TestGroup):
    @testcase(bucket="CopterTests2d")
    def MAVProximity(self):
        '''Test MAVLink proximity driver'''
        self.start_subtest("Test mavlink proximity sensor using DISTANCE_SENSOR messages")  # noqa
        self.set_parameter("SERIAL5_PROTOCOL", 1)
        self.set_parameter("PRX1_TYPE", 2)  # mavlink
        self.reboot_sitl()

        self.progress("Should be unhealthy while we don't send messages")
        self.assert_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_PROXIMITY, True, True, False)

        self.progress("Should be healthy while we're sending good messages")
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > 5:
                raise NotAchievedException("Sensor did not come good")
            self.mav.mav.distance_sensor_send(
                0,  # time_boot_ms
                10, # min_distance cm
                50, # max_distance cm
                20, # current_distance cm
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                21, # id
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE, # orientation
                255  # covariance
            )
            if self.sensor_has_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_PROXIMITY, True, True, True):
                self.progress("Sensor has good state")
                break
            self.delay_sim_time(0.1, reason="poll interval")

        self.progress("Should be unhealthy again if we stop sending messages")
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_PROXIMITY, True, True, False)

        # now make sure we get echoed back the same sorts of things we send:
        # distances are in cm
        distance_map = {
            mavutil.mavlink.MAV_SENSOR_ROTATION_NONE: 30,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_45: 35,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90: 20,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_135: 15,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_180: 70,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_225: 80,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270: 10,
            mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_315: 90,
        }

        wanted_distances = copy.copy(distance_map)
        sensor_enum = mavutil.mavlink.enums["MAV_SENSOR_ORIENTATION"]

        def my_message_hook(mav, m):
            if m.get_type() != 'DISTANCE_SENSOR':
                return
            self.progress("Got (%s)" % str(m))
            want = distance_map[m.orientation]
            got = m.current_distance
            # ArduPilot's floating point conversions make it imprecise:
            delta = abs(want-got)
            if delta > 1:
                self.progress(
                    "Wrong distance (%s): want=%f got=%f" %
                    (sensor_enum[m.orientation].name, want, got))
                return
            if m.orientation not in wanted_distances:
                return
            self.progress(
                "Correct distance (%s): want=%f got=%f" %
                (sensor_enum[m.orientation].name, want, got))
            del wanted_distances[m.orientation]

        self.install_message_hook_context(my_message_hook)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > 5:
                raise NotAchievedException("Sensor did not give right distances")  # noqa
            for (orient, dist) in distance_map.items():
                self.mav.mav.distance_sensor_send(
                    0,  # time_boot_ms
                    10, # min_distance cm
                    90, # max_distance cm
                    dist, # current_distance cm
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                    21, # id
                    orient, # orientation
                    255  # covariance
                )
            self.wait_heartbeat()
            if len(wanted_distances.keys()) == 0:
                break
