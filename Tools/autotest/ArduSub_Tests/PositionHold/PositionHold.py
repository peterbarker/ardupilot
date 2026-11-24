'''
Sub autotest: PositionHold

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase

from ardusub import Joystick


class PositionHold(TestGroup):
    @testcase(bucket="Sub")
    def PositionHold(self):
        """Test POSHOLD mode"""
        self.wait_ready_to_arm()
        self.arm_vehicle()
        # point North
        self.reach_heading_manual(0)
        self.change_mode('POSHOLD')

        # dive a little
        self.set_rc(Joystick.Throttle, 1300)
        self.delay_sim_time(3, reason="vehicle to dive")
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(2, reason="vehicle to settle")

        # Save starting point
        self.assert_receive_message('GLOBAL_POSITION_INT', timeout=5)
        start_pos = self.get_location()
        # Hold in perfect conditions
        self.progress("Testing position hold in perfect conditions")
        self.delay_sim_time(10, reason="position hold measurement period")
        distance_m = self.get_distance(start_pos, self.get_location())
        if distance_m > 1:
            raise NotAchievedException(f"Position Hold was unable to keep position in calm waters within 1 meter after 10 seconds, drifted {distance_m} meters")  # noqa

        # Hold in 1 m/s current
        self.progress("Testing position hold in current")
        self.set_parameter("SIM_WIND_SPD", 1)
        self.set_parameter("SIM_WIND_T", 1)
        self.delay_sim_time(10, reason="drift measurement in 1m/s current")
        distance_m = self.get_distance(start_pos, self.get_location())
        if distance_m > 1:
            raise NotAchievedException(f"Position Hold was unable to keep position in 1m/s current within 1 meter after 10 seconds, drifted {distance_m} meters")  # noqa

        # Move forward slowly in 1 m/s current
        start_pos = self.get_location()
        self.progress("Testing moving forward in position hold in 1m/s current")
        self.set_rc(Joystick.Forward, 1600)
        self.delay_sim_time(10, reason="forward movement")
        distance_m = self.get_distance(start_pos, self.get_location())
        bearing = self.get_bearing(start_pos, self.get_location())
        if distance_m < 2 or (bearing > 30 and bearing < 330):
            raise NotAchievedException(f"Position Hold was unable to move north 2 meters, moved {distance_m} at {bearing} degrees instead")  # noqa
        self.disarm_vehicle()
