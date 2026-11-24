'''
Copter autotest: DefaultsCommaList

AP_FLAKE8_CLEAN
'''

import tempfile

from test_discovery import TestGroup
from test_discovery import testcase


class DefaultsCommaList(TestGroup):
    @testcase(bucket="CopterTests2b")
    def DefaultsCommaList(self):
        '''test that --defaults accepts multiple comma-separated files, with later files overriding earlier ones'''
        f1 = tempfile.NamedTemporaryFile(mode='w', suffix='.parm', delete=False)
        f1.write("RTL_ALT_M 500\nDISARM_DELAY 20\nOA_TYPE 1\nOA_BR_LOOKAHEAD 42\n")
        f1.close()

        f2 = tempfile.NamedTemporaryFile(mode='w', suffix='.parm', delete=False)
        f2.write("RTL_ALT_M 750\n")
        f2.close()

        # wipe: a defaults file only supplies parameters which are not
        # already saved, so with the eeprom left alone anything an
        # earlier test stored wins over the file we are testing -
        # set_autodisarm_delay() saves DISARM_DELAY, and this test then
        # reads back the stored value rather than the one from f1:
        #     DefaultsCommaList (...) (parameter DISARM_DELAY want=20.000000 got=10.000000)
        self.customise_SITL_commandline([],
                                        defaults_filepath=[f1.name, f2.name],
                                        wipe=True)

        # f2 overrides RTL_ALT_M; DISARM_DELAY comes only from f1.
        # The BendyRuler object only exists once OA_TYPE is set, so its
        # parameter must still come from a file that is not the last one
        self.assert_parameter_values({
            "RTL_ALT_M": 750,
            "DISARM_DELAY": 20,
            "OA_BR_LOOKAHEAD": 42,
        })
