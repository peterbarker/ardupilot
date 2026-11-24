'''
Sailboat autotest: DriveRTL

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import PreconditionFailedException

from test_discovery import TestGroup
from test_discovery import testcase


class DriveRTL(TestGroup):
    @testcase(bucket="Sailboat")
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
            m = self.assert_receive_message('MISSION_CURRENT', verbose=True)
            if m.seq == 3:
                break

        self.drain_mav()

        m = self.assert_receive_message('NAV_CONTROLLER_OUTPUT', timeout=1)

        wp_dist_min = 5
        if m.wp_dist < wp_dist_min:
            raise PreconditionFailedException(
                "Did not start at least %f metres from destination (is=%f)" %
                (wp_dist_min, m.wp_dist))

        self.progress("NAV_CONTROLLER_OUTPUT.wp_dist looks good (%u >= %u)" %
                      (m.wp_dist, wp_dist_min,))

        # wait for mission to complete
        self.wait_statustext("Mission Complete", timeout=70)

        # sailboat loiters around RTL point indefinitely:
        self.wait_groundspeed(0.5, 3, timeout=20, minimum_duration=10)

        self.disarm_vehicle()

        self.progress("RTL Mission OK")
