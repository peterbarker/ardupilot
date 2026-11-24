'''
Copter autotest: GroundEffectCompensation_takeOffExpected

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GroundEffectCompensation_takeOffExpected(TestGroup):
    @testcase(bucket="CopterTests2b", disabled="Flapping")
    def GroundEffectCompensation_takeOffExpected(self):
        '''Test EKF's handling of takeoff-expected'''
        self.change_mode('ALT_HOLD')
        self.set_parameters({
            "LOG_FILE_DSRMROT": 1,
            "GNDEFF_TMO": 0,  # release on altitude alone
        })
        self.progress("Making sure we'll have a short log to look at")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()

        # arm the vehicle and let it disarm normally.  This should
        # yield a log where the EKF considers a takeoff imminent until
        # disarm
        self.start_subtest("Check ground effect compensation remains set in EKF while we're at idle on the ground")
        self.arm_vehicle()
        self.wait_disarmed()

        durations = self.get_takeoffexpected_durations_from_current_onboard_log()
        duration = durations[0]
        want = 9
        self.progress("takeoff-expected duration: %fs" % (duration,))
        if duration < want:  # assumes default 10-second DISARM_DELAY
            raise NotAchievedException("Should have been expecting takeoff for longer than %fs (want>%f)" %
                                       (duration, want))

        self.start_subtest("takeoffExpected should be false very soon after we launch into the air")
        self.takeoff(mode='ALT_HOLD', altitude_min=5)
        self.change_mode('LAND')
        self.wait_disarmed()
        durations = self.get_takeoffexpected_durations_from_current_onboard_log(ignore_multi=True)
        self.progress("touchdown-durations: %s" % str(durations))
        duration = durations[0]
        self.progress("takeoff-expected-duration %f" % (duration,))
        want_lt = 5
        if duration >= want_lt:
            raise NotAchievedException("Was expecting takeoff for longer than expected; got=%f want<=%f" %
                                       (duration, want_lt))
