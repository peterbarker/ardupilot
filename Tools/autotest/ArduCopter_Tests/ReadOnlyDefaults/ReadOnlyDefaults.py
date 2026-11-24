'''
Copter autotest: ReadOnlyDefaults

AP_FLAKE8_CLEAN
'''

import tempfile

from test_discovery import TestGroup
from test_discovery import testcase


class ReadOnlyDefaults(TestGroup):
    @testcase(bucket="CopterTests2c")
    def ReadOnlyDefaults(self):
        '''test that defaults marked "readonly" can't be set'''
        defaults_filepath = tempfile.NamedTemporaryFile(mode='w', delete=False)
        defaults_filepath.write("""
DISARM_DELAY 77 @READONLY
RTL_ALT_M 123
RTL_ALT_FINAL_M 129
""")
        defaults_filepath.close()
        # wipe: a defaults file only supplies parameters which are not
        # already saved, so without this whatever an earlier test stored
        # for DISARM_DELAY wins over the @READONLY value being tested:
        #     ReadOnlyDefaults (...) (parameter DISARM_DELAY want=77.000000 got=10.000000)
        self.customise_SITL_commandline([
        ], defaults_filepath=defaults_filepath.name, wipe=True)

        self.context_collect('STATUSTEXT')
        self.send_set_parameter_direct("DISARM_DELAY", 88)

        self.wait_statustext("Param write denied (DISARM_DELAY)")
        self.assert_parameter_value("DISARM_DELAY", 77)
        self.assert_parameter_value("RTL_ALT_M", 123)

        self.start_subtest('Ensure something is writable....')
        self.set_parameter('RTL_ALT_FINAL_M', 101)

        new_values_filepath = tempfile.NamedTemporaryFile(mode='w', delete=False)
        new_values_filepath.write("""
DISARM_DELAY 99
RTL_ALT_M 111
""")
        new_values_filepath.close()

        self.start_subtest("Ensure parameters can't be set via FTP either")
        mavproxy = self.start_mavproxy()
        # can't do two FTP things at once, so wait until parameters are received
        mavproxy.expect("Received .* parameters")
        self.mavproxy_load_module(mavproxy, 'ftp')
        mavproxy.send(f"param ftpload {new_values_filepath.name}\n")
        mavproxy.expect("Loaded")
        self.delay_sim_time(1, reason="parameter write to be processed")
        self.stop_mavproxy(mavproxy)

        self.assert_parameter_value("DISARM_DELAY", 77)
        self.assert_parameter_value("RTL_ALT_M", 111)
        self.assert_parameter_value('RTL_ALT_FINAL_M', 101)
