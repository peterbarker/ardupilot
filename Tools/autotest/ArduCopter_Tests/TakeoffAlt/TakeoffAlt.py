'''
Copter autotest: TakeoffAlt

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TakeoffAlt(TestGroup):
    @testcase(bucket="CopterTests1d")
    def TakeoffAlt(self):
        '''Test Takeoff command altitude'''
        # Test case #1 (set target altitude to relative -10m from the ground, -10m is invalid, so it is set to 1m)
        self.progress("Testing relative alt from the ground")
        self.do_takeoff_alt("copter_takeoff.txt", 1, False)
        # Test case #2 (set target altitude to relative -10m during flight, -10m is invalid, so keeps current altitude)
        self.progress("Testing relative alt during flight")
        self.do_takeoff_alt("copter_takeoff.txt", 10, True)

        self.progress("Takeoff mission completed: passed!")

    def do_takeoff_alt(self, mission_file, target_alt, during_flight=False):
        self.progress("# Load %s" % mission_file)
        # load the waypoint count
        num_wp = self.load_mission(mission_file, strict=False)
        if not num_wp:
            raise NotAchievedException("load %s failed" % mission_file)

        self.set_current_waypoint(1)

        self.change_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        if during_flight:
            self.user_takeoff(alt_min=target_alt)

        # switch into AUTO mode and raise throttle
        self.change_mode("AUTO")
        self.set_rc(3, 1500)

        # fly the mission
        self.wait_waypoint(0, num_wp-1, timeout=500)

        # altitude check
        self.wait_altitude(target_alt - 1 , target_alt + 1, relative=True)

        self.change_mode('LAND')

        # set throttle to minimum
        self.zero_throttle()

        # wait for disarm
        self.wait_disarmed()
        self.progress("MOTORS DISARMED OK")
