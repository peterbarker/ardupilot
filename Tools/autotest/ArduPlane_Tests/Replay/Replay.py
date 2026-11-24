'''
Plane autotest: Replay

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class Replay(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def Replay(self):
        '''test replay correctness'''
        self.progress("Building Replay")
        self.build_replay()
        self.set_parameters({
            "LOG_DARM_RATEMAX": 0,
            "LOG_FILE_RATEMAX": 0,
            "LOG_FILE_BUFSIZE": 32767,
        })

        bits = [
            ('WindAndAirspeed', self.test_replay_wind_and_airspeed_bit),
        ]
        for (name, func) in bits:
            self.start_subtest("%s" % name)
            self.test_replay_bit(func)
