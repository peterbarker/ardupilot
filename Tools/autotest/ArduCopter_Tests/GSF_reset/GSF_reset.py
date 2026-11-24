'''
Copter autotest: GSF_reset

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GSF_reset(TestGroup):
    def gsf_reset_for_ekf_type(self, active_type):
        '''run the GSF emergency-reset sequence with the given EKF active'''
        self.context_push()
        self.set_parameters({
            "AHRS_EKF_TYPE": active_type,
            "EK2_ENABLE": 1,
            "EK3_ENABLE": 1,
            "COMPASS_ORIENT": 4,    # yaw 180
            "COMPASS_USE2": 0,      # disable backup compasses to avoid pre-arm failures
            "COMPASS_USE3": 0,
        })
        self.reboot_sitl()
        self.context_collect('STATUSTEXT')
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()

        # record starting position
        startpos = self.assert_receive_message('LOCAL_POSITION_NED')
        self.progress("startpos=%s" % str(startpos))

        # arm vehicle and takeoff to at least 5m
        self.arm_vehicle()
        expected_alt = 5
        self.user_takeoff(alt_min=expected_alt)

        # watch for emergency yaw reset
        self.wait_statustext("EKF%u IMU. emergency yaw reset" % active_type,
                             timeout=10, regex=True, check_context=True)

        # record how far vehicle flew off
        endpos = self.assert_receive_message('LOCAL_POSITION_NED')
        delta_x = endpos.x - startpos.x
        delta_y = endpos.y - startpos.y
        dist_m = math.sqrt(delta_x*delta_x + delta_y*delta_y)
        self.progress("GSF yaw reset triggered at %f meters" % dist_m)

        self.do_RTL()
        self.context_pop()

        # ensure vehicle did not fly too far
        dist_m_max = 8
        if dist_m > dist_m_max:
            raise NotAchievedException("GSF reset failed, vehicle flew too far (%f > %f)" % (dist_m, dist_m_max))

    @testcase(bucket="CopterTests2d")
    def GSF_reset(self):
        '''test the Gaussian Sum filter based Emergency reset'''
        # the emergency yaw reset request is routed by AHRS to the
        # active estimator only, so fly once with each filter active;
        # the other filter remains enabled throughout.  Each iteration
        # gets its own context so that its statustext collection does
        # not see the previous iteration's messages.
        for active_type in 2, 3:
            self.start_subtest("active EKF type %u" % active_type)
            self.gsf_reset_for_ekf_type(active_type)
