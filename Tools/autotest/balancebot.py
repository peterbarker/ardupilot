'''
Drive a BalanceBot in SITL

AP_FLAKE8_CLEAN

'''

import os

import vehicle_test_suite

from rover import AutoTestRover

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))


class AutoTestBalanceBot(AutoTestRover):

    test_root = "BalanceBot_Tests"

    def log_name(self):
        return "BalanceBot"

    def vehicleinfo_key(self):
        return "Rover"

    def init(self):
        if self.frame is None:
            self.frame = 'balancebot'
        super(AutoTestBalanceBot, self).init()

    def rc_defaults(self):
        ret = super(AutoTestBalanceBot, self).rc_defaults()
        ret[3] = 1500
        return ret

    def is_balancebot(self):
        return True

    def drive_rtl_mission_max_distance_from_home(self):
        '''maximum distance allowed from home at end'''
        '''balancebot tends to wander backwards, away from the target'''
        return 8

    def tests(self):
        '''return list of all tests'''

        '''note that while AutoTestBalanceBot inherits from Rover we don't
inherit Rover's tests!'''
        ret = vehicle_test_suite.TestSuite.tests(self)

        ret.extend(self.discovered_tests("BalanceBot"))
        return ret

    def default_mode(self):
        return 'MANUAL'
