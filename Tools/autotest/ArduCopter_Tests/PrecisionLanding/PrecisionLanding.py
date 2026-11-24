'''
Copter autotest: PrecisionLanding

AP_FLAKE8_CLEAN
'''

from pymavlink import mavextra

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PrecisionLanding(TestGroup):
    @testcase(bucket="CopterTests2a")
    def PrecisionLanding(self):
        """Use PrecLand backends precision messages to land aircraft."""

        self.context_push()

        for backend in [4, 2]:  # SITL, SITL-IRLOCK
            ex = None
            try:
                self.set_parameters({
                    "PLND_ENABLED": 1,
                    "PLND_TYPE": backend,
                })

                self.set_analog_rangefinder_parameters()
                self.set_parameter("SIM_SONAR_SCALE", 12)

                self.set_parameters({
                    "SIM_PLD_ENABLE": 1,
                    "SIM_PLD_HEIGHT": 0,
                    "SIM_PLD_ALT_LMT": 15,
                    "SIM_PLD_DIST_LMT": 10,
                })

                self.reboot_sitl()

                self.progress("Waiting for location")
                self.wait_ready_to_arm()

                # place the target relative to the vehicle's post-reboot
                # position - which is the spawn position.  Sampling the
                # position before the reboot places the target wherever
                # the previous test happened to leave the vehicle:
                start = self.get_location()
                target = start
                (target.lat, target.lng) = mavextra.gps_offset(start.lat, start.lng, 4, -4)
                self.progress("Setting target to %f %f" % (target.lat, target.lng))
                self.set_parameters({
                    "SIM_PLD_LAT": target.lat,
                    "SIM_PLD_LON": target.lng,
                })

                self.zero_throttle()
                self.takeoff(10, 1800, mode="LOITER")
                self.change_mode("LAND")
                self.zero_throttle()
                self.wait_landed_and_disarmed()
                self.assert_receive_message('GLOBAL_POSITION_INT')
                new_pos = self.get_location()
                delta = self.get_distance(target, new_pos)
                self.progress("Landed %f metres from target position" % delta)
                max_delta = 1.5
                if delta > max_delta:
                    raise NotAchievedException("Did not land close enough to target position (%fm > %fm" % (delta, max_delta))

                if not self.current_onboard_log_contains_message("PL"):
                    raise NotAchievedException("Did not see expected PL message")

            except Exception as e:  # noqa: BLE001
                self.print_exception_caught(e)
                ex = e
            self.reboot_sitl()
        self.zero_throttle()
        self.context_pop()
        self.reboot_sitl()
        self.progress("All done")

        if ex is not None:
            raise ex
