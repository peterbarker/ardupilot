'''
Copter autotest: WPArcs

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class WPArcs(TestGroup):
    @testcase(bucket="CopterTests2b")
    def WPArcs(self):
        '''Test WP Arc functionality'''
        _ = self.load_and_start_mission("ArcWPMissionTest.txt")

        self.wait_waypoint(0, 15, max_dist_to_final_wp_m=10, timeout=400)
        self.progress("Check that vehicle flys an arc between WP 15 and 16")
        # A crude check, if the arc waypoint doesn't work then the copter will fly directly south between wp 15 and 16
        # If the arc waypoint works then we expect the copter to arc 180 deg in a CW direction meaning it will travel east
        # of WP 15 before attaining WP 16
        tstart = self.get_sim_time()
        timeout = 200
        last_print_time = 0
        while True:
            now = self.get_sim_time_cached()
            delta = now - tstart
            pos = self.assert_receive_message('GLOBAL_POSITION_INT')
            current_long = pos.lon * 1e-7
            desired_long = 149.16522
            current_wp = self.mav.waypoint_current()
            if delta > timeout:
                raise AutoTestTimeoutException(
                    "Failed to achieve position within %fs" % (timeout,)
                )
            if now - last_print_time > 1:
                self.progress(
                    "Waiting for current long (%f) > desired long (%f) "
                    "(%.2fs remaining before timeout)"
                    % (current_long, desired_long, timeout - delta)
                )
                last_print_time = now

            if current_wp > 19:
                raise NotAchievedException(
                    "mission progressed beyond WP 19 before current longitude > "
                    "desired long (%f)" % desired_long
                )

            if (current_long > desired_long):
                self.progress(
                    "Success: Copter traveled far enough east whilst traveling to "
                    "WP 19 that we are likely flying a WP Arc"
                )
                break

        self.wait_disarmed()
