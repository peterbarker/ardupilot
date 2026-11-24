'''
Copter autotest: AutoTuneYawD

AP_FLAKE8_CLEAN
'''

import vehicle_test_suite

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AutoTuneYawD(TestGroup):
    @testcase(bucket="CopterTests2a")
    def AutoTuneYawD(self):
        """Test autotune mode"""

        # autotune changes a set of parameters on the vehicle which
        # are not in our context.  That changes the flight
        # characteristics, which we can't afford between runs.  So
        # completely reset the simulated vehicle after the run is
        # complete by "customising" the commandline here:
        self.customise_SITL_commandline([])

        self.set_parameters({
            "AUTOTUNE_AGGR": 0.1,
            "AUTOTUNE_AXES": 12,
        })

        gain_names = [
            "ATC_RAT_RLL_D",
            "ATC_RAT_RLL_I",
            "ATC_RAT_RLL_P",
            "ATC_RAT_YAW_D",
        ]
        ogains = self.get_parameters(gain_names)
        # set these parameters so they get reverted at the end of the test:
        self.set_parameters(ogains)

        self.takeoff(10, mode='GUIDED')
        self.hover()
        self.change_mode('STABILIZE')

        tstart = self.get_sim_time()

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        # fail-fast if any determination fails:
        self.install_message_hook_context(vehicle_test_suite.TestSuite.FailFastStatusText(
            self, "determination failed"
        ))
        self.wait_statustext("AutoTune: Success", timeout=5000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))

        # near enough for now:
        self.change_mode('LAND')
        self.wait_landed_and_disarmed()

        self.progress("checking the original gains have been re-instated")
        ngains = self.get_parameters(gain_names)
        for g in ngains.keys():
            if ogains[g] != ngains[g]:
                raise NotAchievedException(f"AUTOTUNE gains not discarded {ogains=} {ngains=}")
