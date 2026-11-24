'''
Copter autotest: OBSTACLE_DISTANCE_3D

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class OBSTACLE_DISTANCE_3D(TestGroup):
    def OBSTACLE_DISTANCE_3D_test_angle(self, angle):
        now = self.get_sim_time_cached()

        distance = 15
        right = distance * math.sin(math.radians(angle))
        front = distance * math.cos(math.radians(angle))
        down = 0

        expected_distance_cm = distance * 100
        # expected orientation
        expected_orientation = int((angle+22.5)/45) % 8
        self.progress("Angle %f expected orient %u" %
                      (angle, expected_orientation))

        tstart = self.get_sim_time()
        last_send = 0
        m = None
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 100:
                raise NotAchievedException("Did not get correct angle back (last-message=%s)" % str(m))

            if now - last_send > 0.1:
                self.progress("ang=%f sending front=%f right=%f" %
                              (angle, front, right))
                self.mav.mav.obstacle_distance_3d_send(
                    int(now*1000),  # time_boot_ms
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                    mavutil.mavlink.MAV_FRAME_BODY_FRD,
                    65535,
                    front,  # x (m)
                    right,  # y (m)
                    down,  # z (m)
                    0,  # min_distance (m)
                    20  # max_distance (m)
                )
                last_send = now
            m = self.mav.recv_match(type="DISTANCE_SENSOR",
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue
            # self.progress("Got (%s)" % str(m))
            if m.orientation != expected_orientation:
                # self.progress("Wrong orientation (want=%u got=%u)" %
                # (expected_orientation, m.orientation))
                continue
            if abs(m.current_distance - expected_distance_cm) > 1:
                # self.progress("Wrong distance (want=%f got=%f)" %
                # (expected_distance_cm, m.current_distance))
                continue
            self.progress("distance-at-angle good")
            break

    @testcase(bucket="CopterTests1c")
    def OBSTACLE_DISTANCE_3D(self):
        '''Check round-trip behaviour of distance sensors'''
        self.context_push()
        self.set_parameters({
            "SERIAL5_PROTOCOL": 1,
            "PRX1_TYPE": 2,
            "SIM_SPEEDUP": 8,  # much GCS interaction
        })
        self.reboot_sitl()
        # need yaw estimate to stabilise:
        self.wait_ekf_happy(require_absolute=True)

        for angle in range(0, 360):
            self.OBSTACLE_DISTANCE_3D_test_angle(angle)

        self.context_pop()
        self.reboot_sitl()
