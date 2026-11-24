'''
Rover autotest: DriveRTL

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import PreconditionFailedException

from test_discovery import TestGroup
from test_discovery import testcase


class DriveRTL(TestGroup):
    def drive_rtl_mission_max_distance_from_home(self):
        '''maximum distance allowed from home at end'''
        return 6.5

    @testcase(bucket="Rover")
    def DriveRTL(self, timeout=120):
        '''Drive an RTL Mission'''
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.load_mission("rtl.txt")
        self.change_mode("AUTO")

        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise AutoTestTimeoutException("Didn't see wp 3")
            m = self.assert_receive_message('MISSION_CURRENT')
            self.progress("MISSION_CURRENT: %s" % str(m))
            if m.seq == 3:
                break

        self.drain_mav()

        m = self.assert_receive_message('NAV_CONTROLLER_OUTPUT')

        wp_dist_min = 5
        if m.wp_dist < wp_dist_min:
            raise PreconditionFailedException(
                "Did not start at least %f metres from destination (is=%f)" %
                (wp_dist_min, m.wp_dist))

        self.progress("NAV_CONTROLLER_OUTPUT.wp_dist looks good (%u >= %u)" %
                      (m.wp_dist, wp_dist_min,))

        # wait for mission to complete
        self.wait_statustext("Mission Complete", timeout=70)

        # the EKF doesn't pull us down to 0 speed:
        self.wait_groundspeed(0, 0.5, timeout=600)

        # current Rover blows straight past the home position and ends
        # up ~6m past the home point.
        home_distance = self.distance_to_home()
        home_distance_min = 5.5
        home_distance_max = self.drive_rtl_mission_max_distance_from_home()
        if home_distance > home_distance_max:
            raise NotAchievedException(
                "Did not stop near home (%f metres distant (%f > want > %f))" %
                (home_distance, home_distance_min, home_distance_max))
        self.disarm_vehicle()
        self.progress("RTL Mission OK (%fm)" % home_distance)
