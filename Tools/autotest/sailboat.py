'''
Drive a Sailboat in SITL

AP_FLAKE8_CLEAN

'''

import os

from rover import AutoTestRover

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))


def log_name(self):
    return "Sailboat"


class AutoTestSailboat(AutoTestRover):

    test_root = "Sailboat_Tests"

    def vehicleinfo_key(self):
        return "Rover"

    def init(self):
        if self.frame is None:
            self.frame = 'sailboat'
        super(AutoTestSailboat, self).init()

    def tests(self):
        '''return list of all tests'''
        ret = ([])

        ret.extend(self.discovered_tests("Sailboat"))
        return ret

    def default_mode(self):
        return 'MANUAL'
