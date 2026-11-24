'''
Tracker autotest: ServoRangeLimits

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ServoRangeLimits(TestGroup):
    @testcase(bucket="Tracker")
    def ServoRangeLimits(self):
        '''check the simulated tracker stops at the limits of its range'''
        self.set_parameters({
            "SERVO1_MAX": 2200,
            "SERVO2_MAX": 2200,
        })
        self.change_mode(0) # "MANUAL"
        # 2200 asks for 1.4 times the model's +/-170 degree yaw range
        # and +/-45 degree pitch range
        for channel in 1, 2:
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                p1=channel,
                p2=2200,
            )
            self.wait_servo_channel_value(channel, 2200)

        self.wait_sim_relative_yaw_pitch(170, 45)

        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 10:
            (yaw, pitch) = self.sim_relative_yaw_pitch()
            if not 169 < yaw < 171:
                raise NotAchievedException(f"Tracker yaw {yaw:.1f} not held at limit")
            if not 44 < pitch < 46:
                raise NotAchievedException(f"Tracker pitch {pitch:.1f} not held at limit")
