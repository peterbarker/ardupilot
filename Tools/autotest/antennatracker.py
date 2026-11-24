'''
Test AntennaTracker vehicle in SITL

AP_FLAKE8_CLEAN

'''

import math
import os

from pymavlink import mavextra

import vehicle_test_suite

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = Location(-27.274439, 151.290064, 343, AltFrame.ABSOLUTE)
SITL_START_HEADING = 8.7


class AutoTestTracker(vehicle_test_suite.TestSuite):

    def log_name(self):
        return "AntennaTracker"

    def test_filepath(self):
        return os.path.realpath(__file__)

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def sitl_start_heading(self):
        return SITL_START_HEADING

    def default_mode(self):
        return "AUTO"

    def is_tracker(self):
        return True

    def default_frame(self):
        return "tracker"

    test_root = "AntennaTracker_Tests"

    def apply_defaultfile_parameters(self):
        # tracker doesn't have a default parameters file
        pass

    def sysid_thismav(self):
        return 2

    def reboot_sitl(self, *args, **kwargs):
        self.disarm_vehicle()
        super(AutoTestTracker, self).reboot_sitl(*args, **kwargs)

    def sim_relative_yaw_pitch(self):
        '''return the simulated tracker's yaw and pitch in degrees,
        relative to the 270 degree yaw and 10 degree pitch at the centre
        of its range'''
        m = self.assert_receive_message('SIMSTATE')
        return (mavextra.wrap_180(math.degrees(m.yaw) - 270),
                math.degrees(m.pitch) - 10)

    def wait_sim_relative_yaw_pitch(self, yaw, pitch, tolerance=1, timeout=120):
        '''wait for the simulated tracker to reach a relative yaw and pitch.
        Pitch moves at only 1 degree/second, so this can take a while'''
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException(f"Did not reach yaw={yaw} pitch={pitch}")
            (current_yaw, current_pitch) = self.sim_relative_yaw_pitch()
            if (abs(mavextra.wrap_180(current_yaw - yaw)) < tolerance and
                    abs(current_pitch - pitch) < tolerance):
                return

    def disabled_tests(self):
        return {
            "ArmFeatures": "See https://github.com/ArduPilot/ardupilot/issues/10652",
            "CPUFailsafe": " tracker doesn't have a CPU failsafe",
        }

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestTracker, self).tests()
        ret.extend([
            self.NMEAOutput,
        ])
        ret.extend(self.discovered_tests("Tracker"))
        return ret
