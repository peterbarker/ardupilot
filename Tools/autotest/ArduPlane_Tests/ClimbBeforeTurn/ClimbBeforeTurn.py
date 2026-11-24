'''
Plane autotest: ClimbBeforeTurn

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ClimbBeforeTurn(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def ClimbBeforeTurn(self):
        '''Test climb-before-turn'''
        self.wait_ready_to_arm()
        self.set_parameters({
            "FLIGHT_OPTIONS": 0,
            "RTL_ALTITUDE": 80,
            "RTL_AUTOLAND": 1,
        })
        takeoff_alt = 10
        self.takeoff(alt=takeoff_alt)
        self.change_mode("CRUISE")
        self.wait_distance_to_home(500, 1000, timeout=60)
        self.change_mode("RTL")
        expected_alt = self.get_parameter("RTL_ALTITUDE")

        home = self.home_position_as_location()
        distance = self.get_distance(home, self.get_location())

        self.wait_altitude(expected_alt - 10, expected_alt + 10, relative=True, timeout=80)

        new_distance = self.get_distance(home, self.get_location())
        # We should be closer to home.
        if new_distance > distance:
            raise NotAchievedException(
                "Expected to be closer to  home (was %fm, now %fm)."
                % (distance, new_distance)
            )

        self.fly_home_land_and_disarm()
        self.set_current_waypoint(0, check_afterwards=False)

        self.change_mode("MANUAL")
        self.set_rc(3, 1000)

        self.wait_ready_to_arm()
        self.set_parameters({
            "FLIGHT_OPTIONS": 16,
            "RTL_ALTITUDE": 100,
        })
        self.takeoff(alt=takeoff_alt)
        self.change_mode("CRUISE")
        self.wait_distance_to_home(500, 1000, timeout=60)
        self.change_mode("RTL")

        home = self.home_position_as_location()
        distance = self.get_distance(home, self.get_location())

        self.wait_altitude(expected_alt - 10, expected_alt + 10, relative=True, timeout=80)

        new_distance = self.get_distance(home, self.get_location())
        # We should be farther from to home.
        if new_distance < distance:
            raise NotAchievedException(
                "Expected to be farther from home (was %fm, now %fm)."
                % (distance, new_distance)
            )

        self.fly_home_land_and_disarm(timeout=240)
