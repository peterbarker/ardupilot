'''
Copter autotest: PayloadPlaceMission

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PayloadPlaceMission(TestGroup):
    @testcase(bucket="CopterTests2c")
    def PayloadPlaceMission(self):
        """Test payload placing in auto."""
        self.set_analog_rangefinder_parameters()
        self.set_servo_gripper_parameters()
        self.reboot_sitl()

        num_wp = self.load_and_start_mission("copter_payload_place.txt")

        self.wait_text("Gripper load releas(ed|ing)", timeout=90, regex=True)
        dist_limit = 1
        # this is a copy of the point in the mission file:
        target_loc = Location.latlon_only(-35.363106, 149.165436)
        dist = self.get_distance(target_loc, self.get_location())
        self.progress("dist=%f" % (dist,))
        if dist > dist_limit:
            raise NotAchievedException("Did not honour target lat/lng (dist=%f want <%f" %
                                       (dist, dist_limit))

        self.wait_waypoint(num_wp-1, num_wp-1)
        self.do_RTL()
