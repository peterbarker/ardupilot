'''
Copter autotest: MaxBotixI2CXL

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MaxBotixI2CXL(TestGroup):
    @testcase(bucket="CopterTests2a")
    def MaxBotixI2CXL(self):
        '''Test maxbotix rangefinder drivers'''
        ex = None
        try:
            self.context_push()

            self.start_subtest("No messages")
            self.assert_not_receiving_message("DISTANCE_SENSOR", timeout=5)

            self.start_subtest("Default address")
            self.set_parameter("RNGFND1_TYPE", 2)  # maxbotix
            self.reboot_sitl()
            self.do_timesync_roundtrip()
            self.assert_receive_message("DISTANCE_SENSOR", timeout=5, verbose=True)

            self.start_subtest("Explicitly set to default address")
            self.set_parameters({
                "RNGFND1_TYPE": 2,  # maxbotix
                "RNGFND1_ADDR": 0x70,
            })
            self.reboot_sitl()
            self.do_timesync_roundtrip()
            rf = self.assert_receive_message("DISTANCE_SENSOR", timeout=5, verbose=True)

            self.start_subtest("Explicitly set to non-default address")
            self.set_parameter("RNGFND1_ADDR", 0x71)
            self.reboot_sitl()
            self.do_timesync_roundtrip()
            rf = self.assert_receive_message("DISTANCE_SENSOR", timeout=5, verbose=True)

            self.start_subtest("Two MaxBotix RangeFinders")
            self.set_parameters({
                "RNGFND1_TYPE": 2,  # maxbotix
                "RNGFND1_ADDR": 0x70,
                "RNGFND1_MIN": 1.50,
                "RNGFND2_TYPE": 2,  # maxbotix
                "RNGFND2_ADDR": 0x71,
                "RNGFND2_MIN": 2.50,
            })
            self.reboot_sitl()
            self.do_timesync_roundtrip()
            for i in [0, 1]:
                rf = self.assert_receive_message(
                    "DISTANCE_SENSOR",
                    timeout=5,
                    condition=f"DISTANCE_SENSOR.id=={i}",
                    verbose=True,
                )
                expected_dist = 150
                if i == 1:
                    expected_dist = 250
                if rf.min_distance != expected_dist:
                    raise NotAchievedException("Unexpected min_cm (want=%u got=%u)" %
                                               (expected_dist, rf.min_distance))

            self.context_pop()
        except Exception as e:  # noqa: BLE001
            self.print_exception_caught(e)
            ex = e

        self.reboot_sitl()
        if ex is not None:
            raise ex
