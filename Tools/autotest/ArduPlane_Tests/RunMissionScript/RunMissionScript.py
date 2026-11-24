'''
Plane autotest: RunMissionScript

AP_FLAKE8_CLEAN
'''

import os

from pysim import util

from test_discovery import TestGroup
from test_discovery import testcase


class RunMissionScript(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def RunMissionScript(self):
        '''Test run_mission.py script'''
        script = os.path.join('Tools', 'autotest', 'run_mission.py')
        self.stop_SITL()
        util.run_cmd([
            util.reltopdir(script),
            self.binary,
            'plane',
            self.generic_mission_filepath_for_filename("flaps.txt"),
        ], checkfail=True)
        self.start_SITL()
        # run_mission.py flew a SITL of its own in this directory, so the
        # storage is not as we left it - notably the parameters the suite
        # relies on, LOG_DISARMED and the two logging rate limits, are no
        # longer set, and nothing here has put them back.  Ask for the
        # standard reset at the end of the test.
        self.context_get().sitl_commandline_customised = True
