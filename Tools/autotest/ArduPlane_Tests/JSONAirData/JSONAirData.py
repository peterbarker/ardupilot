'''
Plane autotest: JSONAirData

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class JSONAirData(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def JSONAirData(self):
        '''check SITL uses the airspeed and wind sent by a JSON physics backend'''
        with self.JSON_physics() as physics:
            # the simulator AHRS reports the wind SIM_JSON received
            self.set_parameters({
                "ARSPD_TYPE": 100,
                "AHRS_EKF_TYPE": 10,
            })
            self.reboot_sitl()

            # wind from the east at 5m/s, then from the north at 6m/s
            for airspeed, wind, direction, speed in (12, [0, -5, 0], 90, 5), (20, [-6, 0, 0], 0, 6):
                physics.set(airspeed=airspeed, velocity_wind=wind)
                self.wait_message_field_values('VFR_HUD', {'airspeed': airspeed}, epsilon=0.5)
                self.wait_message_field_values('WIND', {'direction': direction, 'speed': speed}, epsilon=0.1)
