'''
QuadPlane autotest: HighServoFunctionDefault

AP_FLAKE8_CLEAN
'''

import tempfile

from pymavlink.mavftp import MAVFTP as MavFTP

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class HighServoFunctionDefault(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def HighServoFunctionDefault(self):
        '''check that stale constructor set_default() is purged when param_overrides are loaded'''
        k_motor1 = 33

        defaults_file = tempfile.NamedTemporaryFile(mode='w', delete=False)
        defaults_file.write("SERVO_32_ENABLE 1\n")
        defaults_file.write("SERVO17_FUNCTION %d\n" % k_motor1)
        defaults_file.close()

        # wipe: a defaults file only supplies parameters which are not
        # already saved, so anything an earlier test stored for
        # SERVO17_FUNCTION would win over the default under test
        self.customise_SITL_commandline([], defaults_filepath=defaults_file.name, wipe=True)
        self.assert_parameter_values({"SERVO17_FUNCTION": k_motor1})

        data, _ = self.ftp_burst_read("@PARAM/param.pck?withdefaults=1")

        pdata = MavFTP.ftp_param_decode(bytes(data))
        if pdata is None:
            raise NotAchievedException("param.pck failed to decode")
        if pdata.defaults is None:
            raise NotAchievedException("param.pck has no defaults")

        defaults = {name.decode('utf-8'): value for (name, value, ptype) in pdata.defaults}

        if "SERVO17_FUNCTION" not in defaults:
            raise NotAchievedException("SERVO17_FUNCTION not found in param defaults")

        got = defaults["SERVO17_FUNCTION"]
        if got != k_motor1:
            raise NotAchievedException(
                f"SERVO17_FUNCTION default: expected {k_motor1} got {got}"
            )
