'''
Plane autotest: steplessAHRSSwitch

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class steplessAHRSSwitch(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def steplessAHRSSwitch(self):
        '''Ensure AHRS source switches have input shaping applied correctly'''

        # Set a roll offset
        self.set_parameter("SIM_AHRS_OFF_RLL", 60)

        # Takeoff in TAKEOFF mode
        self.takeoff(alt=50, mode="TAKEOFF")

        # Switch to FBWA
        self.set_rc(3, 1500)
        self.change_mode("FBWA")
        self.delay_sim_time(20, reason="plane to settle")

        # Switch to the offset AHRS source
        self.context_push()
        self.set_parameter("AHRS_EKF_TYPE", 10)

        self.delay_sim_time(10, reason="Wait for vehicle to recover")

        # Revert to original AHRS source
        self.context_pop()

        self.delay_sim_time(10, reason="Wait for vehicle to recover")

        self.fly_home_land_and_disarm()

        # Check the roll error in the log in FBWA
        dfreader = self.dfreader_for_current_onboard_log()
        max_roll_error = 0.0
        mode = None
        FBWA_mode_num = 5
        while True:
            m = dfreader.recv_match(type=['ATIS', 'MODE'])
            if m is None:
                break
            if m.get_type() == 'MODE':
                mode = m.ModeNum
                continue
            if mode == FBWA_mode_num:
                max_roll_error = max(max_roll_error, abs(m.rErr))

        # If input shaping was not working correctly we would see an error nearly as large as the roll offset
        roll_threshold = 15
        if max_roll_error > roll_threshold:
            raise NotAchievedException("Large roll error %0.1f > %0.1f" % (max_roll_error, roll_threshold))

        self.progress("Roll error check passed %0.1f <= %0.1f" % (max_roll_error, roll_threshold))
