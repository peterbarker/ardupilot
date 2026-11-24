'''
Copter autotest: RangeFinderPowerDown

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class RangeFinderPowerDown(TestGroup):
    @testcase(bucket="CopterTests2b")
    def RangeFinderPowerDown(self):
        '''Test rangefinder power down by altitude for multiple backends'''
        pwrrng = 10  # power down above this altitude in metres

        self.install_terrain_handlers_context()

        backends = [
            ("SITL", {"RNGFND1_TYPE": 100}),
            ("TFMiniPlus", {"RNGFND1_TYPE": 25, "RNGFND1_ADDR": 0x09}),
        ]

        rf_bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION
        low_alt = pwrrng - 3   # below threshold
        high_alt = pwrrng + 5  # above threshold

        for (name, backend_params) in backends:
            self.start_subtest(f"RangeFinderPowerDown: {name}")
            self.context_push()

            params = {
                "RNGFND1_MAX": 50,  # prevent unhealthy due to max range limit
                "RNGFND1_PWRRNG": pwrrng,
            }
            params.update(backend_params)
            self.set_parameters(params)
            self.reboot_sitl()

            self.takeoff(low_alt, mode='GUIDED')

            self.progress("Verify rangefinder healthy below power-down threshold")
            self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
            self.context_set_message_rate_hz('RANGEFINDER', self.sitl_streamrate())
            self.assert_rangefinder_distance_between(low_alt - 2, low_alt + 2)

            self.progress(f"Climbing above power-down threshold ({pwrrng}m)")
            self.fly_guided_move_local(0, 0, high_alt)

            self.progress("Verify rangefinder powered down above threshold")
            self.wait_sensor_state(rf_bit, present=True, enabled=True, healthy=False)

            self.progress("Descending below power-down threshold")
            self.fly_guided_move_local(0, 0, low_alt)

            self.progress("Verify rangefinder recovers below threshold")
            self.wait_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
            self.assert_rangefinder_distance_between(low_alt - 2, low_alt + 2)

            self.land_and_disarm()
            self.context_pop()
