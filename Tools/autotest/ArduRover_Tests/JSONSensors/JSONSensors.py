'''
Rover autotest: JSONSensors

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import quaternion

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class JSONSensors(TestGroup):
    @testcase(bucket="Rover")
    def JSONSensors(self):
        '''check SITL uses the sensor data sent by a JSON physics backend'''
        with self.JSON_physics() as physics:
            physics.set(attitude=[0, 0, math.radians(90)])
            self.set_parameters({
                "RNGFND1_TYPE": 100,
                "BATT_MONITOR": 4,
                "WNDVN_TYPE": 11,
                "WNDVN_SPEED_TYPE": 11,
            })
            self.reboot_sitl()
            self.wait_heading(90)
            self.set_message_rate_hz('WIND', 10)

            # two sets of values, so each field is seen to follow a change
            for rng, volt, amp, wind_dir, wind_spd in (3.5, 11.1, 2.5, 45, 5), (6.0, 12.3, 4.0, 120, 8):
                physics.set(rng_1=rng,
                            battery={'voltage': volt, 'current': amp},
                            windvane={'direction': math.radians(wind_dir), 'speed': wind_spd})
                self.wait_rangefinder_distance(rng - 0.1, rng + 0.1)
                self.wait_message_field_values('SYS_STATUS', {
                    'voltage_battery': volt * 1000,
                    'current_battery': amp * 100,
                }, epsilon=20)
                self.wait_and_maintain("AppWndDir", wind_dir,
                                       lambda: self.assert_receive_named_value_float('AppWndDir').value,
                                       accuracy=1)
                self.wait_and_maintain("AppWndSpd", wind_spd,
                                       lambda: self.assert_receive_named_value_float('AppWndSpd').value,
                                       accuracy=0.1)

            # the backend moves the vehicle east, then north
            self.wait_ready_to_arm()
            for velocity, speed in ([0, 1, 0], 1), ([2, 0, 0], 2):
                physics.set(velocity=velocity)
                self.wait_groundspeed(speed - 0.1, speed + 0.1)
                start = self.get_location()
                self.delay_sim_time(10, reason="vehicle moving")
                dist = self.get_distance(start, self.get_location())
                if abs(dist - 10 * speed) > 1:
                    raise NotAchievedException("moved %fm in 10s, expected %fm" % (dist, 10 * speed))
            physics.set(velocity=[0, 0, 0])

            # euler attitude, then quaternion which takes precedence over it
            for field, euler in (('attitude', (-5, 0, -90)),
                                 ('quaternion', (10, 0, 90)),
                                 ('quaternion', (-20, 5, 45))):
                roll, pitch, yaw = [math.radians(a) for a in euler]
                if field == 'quaternion':
                    physics.set(quaternion=quaternion.Quaternion([roll, pitch, yaw]).q.tolist())
                else:
                    physics.set(attitude=[roll, pitch, yaw])
                self.wait_message_field_values('SIMSTATE', {
                    'roll': roll,
                    'pitch': pitch,
                    'yaw': yaw,
                }, epsilon=0.01)
