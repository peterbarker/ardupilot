'''
Copter autotest: AltEstimation

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AltEstimation(TestGroup):
    @testcase(bucket="CopterTests1c")
    def AltEstimation(self):
        '''Test that Alt Estimation is mandatory for ALT_HOLD'''
        # disable barometer so there is no altitude source
        self.set_parameters({
            "SIM_BARO_DISABLE": 1,
            "SIM_BAR2_DISABLE": 1,
        })

        self.wait_gps_disable(position_vertical=True)

        # turn off arming checks (mandatory arming checks will still be run)
        self.set_parameter("ARMING_SKIPCHK", -1)

        # delay 12 sec to allow EKF to lose altitude estimate
        self.delay_sim_time(12, reason="EKF to lose altitude estimate")

        self.change_mode("ALT_HOLD")
        self.assert_prearm_failure("Need Alt Estimate")

        # arm vehicle in stabilize to bypass barometer pre-arm checks
        self.change_mode("STABILIZE")
        self.arm_vehicle()
        self.set_rc(3, 1700)
        try:
            self.change_mode("ALT_HOLD", timeout=10)
        except AutoTestTimeoutException:
            self.progress("PASS not able to set mode without Position : %s" % "ALT_HOLD")

        # check that mode change to ALT_HOLD has failed (it should)
        if self.mode_is("ALT_HOLD"):
            raise NotAchievedException("Changed to ALT_HOLD with no altitude estimate")
        self.disarm_vehicle(force=True)

        # we are not at the home location - reboot so the next test starts there
        self.reboot_sitl()
