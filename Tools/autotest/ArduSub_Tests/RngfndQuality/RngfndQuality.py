'''
Sub autotest: RngfndQuality

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RngfndQuality(TestGroup):
    @testcase(bucket="Sub")
    def RngfndQuality(self):
        """Check lua Range Finder quality information flow"""
        self.context_collect('STATUSTEXT')

        self.set_parameters({
            "SCR_ENABLE": 1,
            "RNGFND1_TYPE": 36,
            "RNGFND1_ORIENT": 25,
            "RNGFND1_MIN": 0.10,
            "RNGFND1_MAX": 50.00,
        })

        self.install_example_script_context("rangefinder_quality_test.lua")

        # These string must match those sent by the lua test script.
        complete_str = "#complete#"
        failure_str = "!!failure!!"

        self.reboot_sitl()

        self.wait_statustext(complete_str, timeout=20, check_context=True)
        found_failure = self.statustext_in_collections(failure_str)

        if found_failure is not None:
            raise NotAchievedException(f"RngfndQuality test failed: {found_failure.text}")
