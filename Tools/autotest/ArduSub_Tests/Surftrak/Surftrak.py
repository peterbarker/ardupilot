'''
Sub autotest: Surftrak

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase

from ardusub import Joystick


class Surftrak(TestGroup):
    def watch_distance_maintained(self, delta=0.3, timeout=5.0):
        """Watch and wait for the rangefinder reading to be maintained"""
        tstart = self.get_sim_time_cached()
        self.context_push()
        self.context_set_message_rate_hz('RANGEFINDER', self.sitl_streamrate())
        previous_distance = self.assert_receive_message('RANGEFINDER').distance
        previous_distance_ds = self.assert_receive_message('DISTANCE_SENSOR').current_distance * 0.01  # cm -> m
        self.progress(f'Distance to be watched: {previous_distance:.2f}')
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                self.progress(f'Distance hold done: {previous_distance}')
                self.context_pop()
                return
            m = self.assert_receive_message('RANGEFINDER')
            if abs(m.distance - previous_distance) > delta:
                raise NotAchievedException(
                    f"Distance not maintained: want {previous_distance:.2f} (+/- {delta:.2f}) got={m.distance:.2f}")
            m = self.assert_receive_message('DISTANCE_SENSOR')
            if abs(m.current_distance*0.01 - previous_distance_ds) > delta:
                raise NotAchievedException(
                    f"Distance not maintained: want {previous_distance:.2f} (+/- {delta:.2f}) got={m.distance:.2f}")

    @testcase(bucket="Sub")
    def Surftrak(self):
        """Test SURFTRAK mode"""

        self.assert_parameter_value('RNGFND1_MAX', 30)

        # Something closer to Bar30 noise
        self.set_parameter("SIM_BARO_RND", 0.01)

        # Dive to -5m, outside of rangefinder range, will act like ALT_HOLD
        self.dive(-5, timeout=60)
        self.context_collect('STATUSTEXT')
        self.change_mode(21)
        self.wait_statustext('waiting for a rangefinder reading', check_context=True)
        self.context_clear_collection('STATUSTEXT')
        self.watch_altitude_maintained()

        # Move into range, should set a rangefinder target and maintain it
        self.set_rc(Joystick.Throttle, 1300)
        self.wait_altitude(altitude_min=-26, altitude_max=-25, relative=False, timeout=60)
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(4, reason="rangefinder target to settle")
        self.wait_statustext('rangefinder target is', check_context=True)
        self.context_clear_collection('STATUSTEXT')
        self.watch_distance_maintained()

        # Move a few meters, should apply a delta and maintain the new rangefinder target
        self.set_rc(Joystick.Throttle, 1300)
        self.wait_altitude(altitude_min=-31, altitude_max=-30, relative=False, timeout=60)
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(4, reason="rangefinder target to settle")
        self.wait_statustext('rangefinder target is', check_context=True)
        self.watch_distance_maintained()

        # Disarm, allowing the vehicle to drift up
        self.disarm_vehicle()
        self.delay_sim_time(5, reason="vehicle to drift upward")

        # Re-arm. The vehicle should get a new rangefinder target and maintain distance
        self.arm_vehicle()
        self.watch_distance_maintained()

        self.disarm_vehicle()
