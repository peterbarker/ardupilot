'''
Rover autotest: AP_Proximity_MAV

AP_FLAKE8_CLEAN
'''

import time

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AP_Proximity_MAV(TestGroup):
    def mavlink_time_boot_us(self):
        '''returns a time suitable for putting into the time_boot_ms entry in mavlink packets'''
        return int(time.time() * 1000000000)

    def ap_proximity_mav_obstacle_distance_send(self, data):
        increment = data.get("increment", 0)
        increment_f = data.get("increment_f", 0.0)
        max_distance = data["max_distance"]
        invalid_distance = max_distance + 1  # per spec
        distances = data["distances"][:]
        distances.extend([invalid_distance] * (72-len(distances)))
        self.mav.mav.obstacle_distance_send(
            self.mavlink_time_boot_us(),
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            distances,
            increment,
            data["min_distance"],
            data["max_distance"],
            increment_f,
            data["angle_offset"],
            mavutil.mavlink.MAV_FRAME_BODY_FRD
        )

    def send_obstacle_distances_expect_distance_sensor_messages(self, obstacle_distances_in, expect_distance_sensor_messages):
        self.delay_sim_time(11, reason="obstacles to time out")  # allow obstacles to time out
        self.do_timesync_roundtrip()
        expect_distance_sensor_messages_copy = expect_distance_sensor_messages[:]
        last_sent = 0
        while True:
            now = self.get_sim_time_cached()
            if now - last_sent > 1:
                self.progress("Sending")
                self.ap_proximity_mav_obstacle_distance_send(obstacle_distances_in)
                last_sent = now
            m = self.mav.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1)
            self.progress("Got (%s)" % str(m))
            if m is None:
                self.delay_sim_time(1, reason="sensor message poll interval")
                continue
            orientation = m.orientation
            found = False
            if m.current_distance == m.max_distance:
                # ignored
                continue
            for expected_distance_sensor_message in expect_distance_sensor_messages_copy:
                if expected_distance_sensor_message["orientation"] != orientation:
                    continue
                found = True
                if not expected_distance_sensor_message.get("__found__", False):
                    self.progress("Marking message as found")
                    expected_distance_sensor_message["__found__"] = True
                if (m.current_distance - expected_distance_sensor_message["distance"] > 1):
                    raise NotAchievedException(
                        "Bad distance for orient=%u want=%u got=%u" %
                        (orientation, expected_distance_sensor_message["distance"], m.current_distance))
                break
            if not found:
                raise NotAchievedException("Got unexpected DISTANCE_SENSOR message")
            all_found = True
            for expected_distance_sensor_message in expect_distance_sensor_messages_copy:
                if not expected_distance_sensor_message.get("__found__", False):
                    self.progress("message still not found (orient=%u" % expected_distance_sensor_message["orientation"])
                    all_found = False
                    break
            if all_found:
                self.progress("Have now seen all expected messages")
                break

    @testcase(bucket="Rover")
    def AP_Proximity_MAV(self):
        '''Test MAV proximity backend'''

        self.set_parameters({
            "PRX1_TYPE": 2,  # AP_Proximity_MAV
            "OA_TYPE": 2,  # dijkstra
            "OA_DB_OUTPUT": 3,  # send all items
        })
        self.reboot_sitl()

        # 1 laser pointing straight forward:
        self.send_obstacle_distances_expect_distance_sensor_messages(
            {
                "distances": [234],
                "increment_f": 10,
                "angle_offset": 0.0,
                "min_distance": 0,
                "max_distance": 1000, # cm
            }, [
                {"orientation": 0, "distance": 234},
            ])

        # 5 lasers at front of vehicle, spread over 40 degrees:
        self.send_obstacle_distances_expect_distance_sensor_messages(
            {
                "distances": [111, 222, 333, 444, 555],
                "increment_f": 10,
                "angle_offset": -20.0,
                "min_distance": 0,
                "max_distance": 1000, # cm
            }, [
                {"orientation": 0, "distance": 111},
            ])

        # lots of dense readings (e.g. vision camera:
        distances = [0] * 72
        for i in range(0, 72):
            distances[i] = 1000 + 10*abs(36-i)

        self.send_obstacle_distances_expect_distance_sensor_messages(
            {
                "distances": distances,
                "increment_f": 90/72.0,
                "angle_offset": -45.0,
                "min_distance": 0,
                "max_distance": 2000, # cm
            }, [
                {"orientation": 0, "distance": 1000},
                {"orientation": 1, "distance": 1190},
                {"orientation": 7, "distance": 1190},
            ])
