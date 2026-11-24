'''
Plane autotest: LastLetterLoiter

AP_FLAKE8_CLEAN
'''

import os

from test_discovery import TestGroup
from test_discovery import testcase

from arduplane import testdir


class LastLetterLoiter(TestGroup):
    @testcase(bucket="LastLetterPlane")
    def LastLetterLoiter(self):
        '''Test Loiter mode with the last_letter external simulator'''
        self.customise_SITL_commandline(
            [],
            model="last_letter",
            defaults_filepath=self.model_defaults_filepath("last_letter") + [
                # The suite's default frame is plane-elevrev, which reverses
                # the pilot's pitch and yaw input. The last_letter frame does
                # not carry the reversal, so add it here rather than forking
                # the profile.
                os.path.join(testdir, "models", "plane-reverse-rc-pitch-yaw.parm"),
            ],
            wipe=True,
        )
        self.LOITER()
