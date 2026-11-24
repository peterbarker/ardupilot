'''
Helicopter autotest: AutorotationPreArm

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AutorotationPreArm(TestGroup):
    @testcase(bucket="Helicopter")
    def AutorotationPreArm(self):
        """Check autorotation pre-arms are working"""
        self.context_push()
        self.start_subtest("Check pass when autorotation mode not enabled")
        self.set_parameters({
            "AROT_ENABLE": 0,
            "RPM1_TYPE": 0
        })
        self.reboot_sitl()
        try:
            self.wait_statustext("PreArm: AROT: RPM1 not enabled", timeout=50)
            raise NotAchievedException("Received AROT prearm when not AROT not enabled")
        except AutoTestTimeoutException:
            # We want to hit the timeout on wait_statustext()
            pass

        self.start_subtest("Check pre-arm fails when autorotation mode enabled")
        self.set_parameter("AROT_ENABLE", 1)
        self.wait_statustext("PreArm: AROT: RPM1 not enabled", timeout=50)
        self.set_parameter("RPM1_TYPE", 10) # reboot required to take effect
        self.reboot_sitl()

        self.start_subtest("Check pre-arm fails with bad HS_Sensor config")
        self.context_push()
        self.set_parameter("AROT_HS_SENSOR", -1)
        self.wait_statustext("PreArm: AROT: RPM instance <0", timeout=50)
        self.context_pop()

        self.start_subtest("Check pre-arm fails with bad RSC config")
        self.wait_statustext("PreArm: AROT: H_RSC_AROT_* not configured", timeout=50)

        self.start_subtest("Check pre-arms clear with all issues corrected")
        self.set_parameter("H_RSC_AROT_ENBL", 1)
        self.wait_ready_to_arm()

        self.context_pop()
