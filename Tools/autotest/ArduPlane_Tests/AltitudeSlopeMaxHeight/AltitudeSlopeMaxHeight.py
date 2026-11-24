'''
Plane autotest: AltitudeSlopeMaxHeight

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AltitudeSlopeMaxHeight(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def AltitudeSlopeMaxHeight(self):
        '''Test rebuild altitude slope if above and climbing'''

        # Test that ALT_SLOPE_MAXHGT correctly controls re-planning altitude slope
        # in the scenario that aircraft is above planned slope and slope is positive (climbing).
        #
        #
        #  Behaviour with ALT_SLOPE_MAXHGT = 0 (no slope replanning)
        #       (2)..      __(4)
        #         |  \..__/
        #         |  __/
        #         (3)
        #
        # Behaviour with ALT_SLOPE_MAXHGT = 5 (slope replanning when >5m error)
        #       (2)........__(4)
        #         |     __/
        #         |  __/
        #         (3)
        # Solid is plan, dots are actual flightpath.

        self.load_mission('rapid-descent-then-climb.txt', strict=False)

        self.set_current_waypoint(1)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        #
        # Initial run with ALT_SLOPE_MAXHGT = 5 (default).
        #
        self.set_parameter("ALT_SLOPE_MAXHGT", 5)

        # Wait for waypoint commanding rapid descent, followed by climb.
        self.wait_current_waypoint(5, timeout=1200)

        # Altitude should not descend significantly below the initial altitude
        init_altitude = self.get_altitude(relative=True, timeout=2)
        timeout = 600
        wpnum = 7
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > timeout:
                raise AutoTestTimeoutException("Did not get wanted current waypoint")

            if (self.get_altitude(relative=True, timeout=2) - init_altitude) < -10:
                raise NotAchievedException("Descended >10m before reaching desired waypoint,\
  indicating slope was not replanned")

            seq = self.mav.waypoint_current()
            self.progress("Waiting for wp=%u current=%u" % (wpnum, seq))
            if seq == wpnum:
                break

        self.set_current_waypoint(2)

        #
        # Second run with ALT_SLOPE_MAXHGT = 0 (no re-plan).
        #
        self.set_parameter("ALT_SLOPE_MAXHGT", 0)

        # Wait for waypoint commanding rapid descent, followed by climb.
        self.wait_current_waypoint(5, timeout=1200)

        # This time altitude should descend significantly below the initial altitude
        init_altitude = self.get_altitude(relative=True, timeout=2)
        timeout = 600
        wpnum = 7
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > timeout:
                raise AutoTestTimeoutException("Did not get wanted altitude")

            seq = self.mav.waypoint_current()
            self.progress("Waiting for wp=%u current=%u" % (wpnum, seq))
            if seq == wpnum:
                raise NotAchievedException("Reached desired waypoint without first descending 10m,\
 indicating slope was replanned unexpectedly")

            if (self.get_altitude(relative=True, timeout=2) - init_altitude) < -10:
                break

        # Disarm
        self.wait_disarmed(timeout=600)

        self.progress("Mission OK")
