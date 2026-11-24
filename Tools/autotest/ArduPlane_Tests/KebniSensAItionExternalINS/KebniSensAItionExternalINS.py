'''
Plane autotest: KebniSensAItionExternalINS

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class KebniSensAItionExternalINS(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def KebniSensAItionExternalINS(self):
        '''Test Kebni SensAItion External INS mode. Or in Ardupilot terminology, ExternalAHRS mode.'''
        self.set_parameters({
            "EAHRS_OPTIONS": 4  # INS mode -> Bit 2 set
        })
        # This test is sensitive to SITL::Plane's update() loop timing, which manifests as landing
        # quite far from the final target spot. Ideally, this increased threshold will be removed
        # in the future when the test is more robust.
        self.fly_external_AHRS("SensAItionINS", 11,
                               dist_to_final_wp_threshold_m=75.0)
