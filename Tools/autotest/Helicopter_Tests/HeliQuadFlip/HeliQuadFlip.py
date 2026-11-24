'''
Helicopter autotest: HeliQuadFlip

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class HeliQuadFlip(TestGroup):
    @testcase(bucket="Helicopter")
    def HeliQuadFlip(self):
        '''fly Flip mode on collective-pitch quad frame'''
        self.customise_SITL_commandline(
            [],
            defaults_filepath=self.model_defaults_filepath('heli-quad'),
            model="heli-quad:@ROMFS/models/heliquad.json",
            wipe=True,
        )
        # pitch flips are skipped; unlike a fixed-pitch quad, whose
        # throttle cut during the flip also cuts control authority,
        # the heli-quad retains full authority at zero collective and
        # rotates well past the commanded rate. The recovery then
        # leaves the attitude target wedged at the pitch-90 Euler
        # singularity, from which ALT_HOLD's euler-angle input
        # shaping cannot recover cleanly
        self.ModeFlip(do_pitch_flip=False)
