'''
Copter autotest: MAVLinkRangeFinderIDs

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAVLinkRangeFinderIDs(TestGroup):
    @testcase(bucket="CopterTests1d")
    def MAVLinkRangeFinderIDs(self):
        '''test multiple MAVLink rangefinders selected by DISTANCE_SENSOR id'''
        self.context_push()
        try:
            self.set_parameters({
                "SERIAL5_PROTOCOL": 1,
                "RNGFND1_TYPE": 10,
                "RNGFND1_ADDR": 1,
                "RNGFND2_TYPE": 10,
                "RNGFND2_ADDR": 2,
            })
            self.reboot_sitl()

            # we are interacting with the autopilot, reduce chance of
            # hitting timeouts on supplied data:
            self.context_set_speedup(1)
            self.context_set_message_rate_hz("DISTANCE_SENSOR", 10)

            self.context_collect("DISTANCE_SENSOR")
            input_distances = {
                1: 20,
                2: 30,
            }
            for _ in range(10):
                for input_id, distance_cm in input_distances.items():
                    self.mav.mav.distance_sensor_send(
                        0,  # time_boot_ms
                        10, # min_distance
                        50, # max_distance
                        distance_cm, # current_distance
                        mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                        input_id, # id
                        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # orientation
                        255 # covariance
                    )
                self.delay_sim_time(0.1, reason="collect rangefinder output")

            messages = self.context_collection("DISTANCE_SENSOR")
            self.context_stop_collecting("DISTANCE_SENSOR")
            if not messages:
                raise NotAchievedException("Did not receive DISTANCE_SENSOR output")

            output_distances = {
                0: input_distances[1],
                1: input_distances[2],
            }
            seen_ids = set()
            for message in messages:
                if message.id not in output_distances:
                    raise NotAchievedException(
                        "Unexpected MAVLink rangefinder backend id %u" % message.id)
                distance_cm = output_distances[message.id]
                if abs(message.current_distance - distance_cm) > 1:
                    raise NotAchievedException(
                        "MAVLink rangefinder distance mismatch "
                        "(backend=%u want=%u got=%u)" %
                        (message.id, distance_cm, message.current_distance))
                seen_ids.add(message.id)

            if seen_ids != set(output_distances.keys()):
                raise NotAchievedException(
                    "Did not receive output from all MAVLink rangefinder backends "
                    "(want=%s got=%s)" %
                    (sorted(output_distances.keys()), sorted(seen_ids)))
        finally:
            self.context_pop()
