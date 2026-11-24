'''
Copter autotest: ModelDefaults

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ModelDefaults(TestGroup):
    @testcase(bucket="CopterTests2a")
    def ModelDefaults(self):
        '''check SITL loads the default parameters for each --model'''
        # firmware defaults: FRAME_CLASS 0, FRAME_TYPE 1, ACRO_RP_RATE 360, ACRO_Y_RATE 202.5
        callisto = {"FRAME_CLASS": 4, "FRAME_TYPE": 14, "ACRO_Y_RATE": 90}
        for model, expected in [
                ("quad", {"FRAME_CLASS": 1, "FRAME_TYPE": 0}),
                ("X", {"FRAME_CLASS": 1, "FRAME_TYPE": 1}),
                # a frame name selects the frame's own model
                ("Callisto", callisto),
                ("freestyle", {"FRAME_CLASS": 1, "FRAME_TYPE": 1, "ACRO_RP_RATE": 533}),
                # a frame's model string
                ("octa-quad-cwx:@ROMFS/models/Callisto.json", callisto),
                # ":<args>" is dropped to find the frame
                ("quad:@ROMFS/models/freestyle.json", {"FRAME_CLASS": 1, "FRAME_TYPE": 0, "ACRO_RP_RATE": 360}),
        ]:
            self.start_subtest(model)
            self.customise_SITL_commandline([], model=model, wipe=True)
            self.assert_parameter_values(expected)
