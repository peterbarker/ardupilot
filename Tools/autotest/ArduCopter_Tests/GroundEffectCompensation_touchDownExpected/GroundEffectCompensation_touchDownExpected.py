'''
Copter autotest: GroundEffectCompensation_touchDownExpected

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GroundEffectCompensation_touchDownExpected(TestGroup):
    @testcase(bucket="CopterTests2b", disabled="Flapping")
    def GroundEffectCompensation_touchDownExpected(self):
        '''Test EKF's handling of touchdown-expected'''
        self.zero_throttle()
        self.change_mode('ALT_HOLD')
        self.set_parameters({
            "LOG_FILE_DSRMROT": 1,
            "GNDEFF_ALT": 0,  # no touchdown altitude gate: whole descent counts
        })
        self.progress("Making sure we'll have a short log to look at")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()

        self.start_subtest("Make sure touchdown-expected duration is about right")
        self.takeoff(20, mode='ALT_HOLD')
        self.change_mode('LAND')
        self.wait_disarmed()

        durations = self.get_touchdownexpected_durations_from_current_onboard_log(ignore_multi=True)
        self.progress("touchdown-durations: %s" % str(durations))
        duration = durations[-1]
        expected = 23  # this is the time in the final descent phase of LAND
        if abs(duration - expected) > 5:
            raise NotAchievedException("Was expecting roughly %fs of touchdown expected, got %f" % (expected, duration))

        # we are not at the home location - reboot so the next test starts there
        self.reboot_sitl()
