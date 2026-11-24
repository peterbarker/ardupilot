'''
Tracker autotest: UndrivenServosHoldPosition

AP_FLAKE8_CLEAN
'''

from pymavlink import mavextra

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class UndrivenServosHoldPosition(TestGroup):
    @testcase(bucket="Tracker")
    def UndrivenServosHoldPosition(self):
        '''check the simulated tracker does not move while its servos are not driven'''
        self.change_mode(0) # "MANUAL"
        # centre the tracker so that it has room to move in either direction
        self.set_rc(1, 1500)
        self.set_rc(2, 1500)
        self.wait_sim_relative_yaw_pitch(0, 0)
        self.disarm_vehicle()
        for channel in 1, 2:
            self.wait_servo_channel_value(channel, 0)
        (start_yaw, start_pitch) = self.sim_relative_yaw_pitch()
        self.delay_sim_time(12, reason="tracker to hold position")
        (end_yaw, end_pitch) = self.sim_relative_yaw_pitch()
        self.progress(f"yaw {start_yaw:.1f} -> {end_yaw:.1f}, pitch {start_pitch:.1f} -> {end_pitch:.1f}")
        if abs(mavextra.wrap_180(end_yaw - start_yaw)) > 1:
            raise NotAchievedException("Tracker yawed while undriven")
        if abs(end_pitch - start_pitch) > 1:
            raise NotAchievedException("Tracker pitched while undriven")
        self.arm_vehicle()
