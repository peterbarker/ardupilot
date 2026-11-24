'''
Sub autotest: MotorThrustHoverParameterIgnore

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MotorThrustHoverParameterIgnore(TestGroup):
    @testcase(bucket="Sub")
    def MotorThrustHoverParameterIgnore(self):
        """Test if we are ignoring MOT_THST_HOVER parameter"""

        # Test default parameter value
        mot_thst_hover_value = self.get_parameter("MOT_THST_HOVER")
        if mot_thst_hover_value != 0.5:
            raise NotAchievedException(f"Unexpected default MOT_THST_HOVER parameter value {mot_thst_hover_value}")

        # Test if parameter is being ignored
        for value in [0.25, 0.75]:
            self.set_parameter("MOT_THST_HOVER", value)
            self.AltitudeHold()
