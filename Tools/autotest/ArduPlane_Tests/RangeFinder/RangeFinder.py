'''
Plane autotest: RangeFinder

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RangeFinder(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def RangeFinder(self):
        '''Test RangeFinder Basic Functionality'''
        self.progress("Making sure we don't ordinarily get RANGEFINDER")
        self.assert_not_receive_message('RANGEFINDER')
        self.assert_not_receive_message('DISTANCE_SENSOR')

        self.set_analog_rangefinder_parameters()

        self.reboot_sitl()

        self.context_set_message_rate_hz('RANGEFINDER', self.sitl_streamrate())

        '''ensure rangefinder gives height-above-ground'''
        self.load_mission("plane-gripper-mission.txt") # borrow this
        self.set_parameter("RTL_AUTOLAND", 1)
        self.set_current_waypoint(1)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_waypoint(5, 5, max_dist_to_final_wp_m=100)
        rf = self.assert_receive_message('RANGEFINDER')
        ds = self.assert_receive_message('DISTANCE_SENSOR')
        gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
        if abs(rf.distance - gpi.relative_alt/1000.0) > 3:
            raise NotAchievedException(
                "rangefinder alt (%s) disagrees with global-position-int.relative_alt (%s)" %
                (rf.distance, gpi.relative_alt/1000.0))
        if abs(ds.current_distance*0.01 - gpi.relative_alt/1000.0) > 3:
            raise NotAchievedException(
                "distance_sensor alt (%s) disagrees with global-position-int.relative_alt (%s)" %
                (ds.current_distance*0.01, gpi.relative_alt/1000.0))
        self.wait_statustext("Auto disarmed", timeout=60)

        self.progress("Ensure RFND messages in log")
        if not self.current_onboard_log_contains_message("RFND"):
            raise NotAchievedException("No RFND messages in log")
