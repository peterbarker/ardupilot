'''
Copter autotest: EKFYawResetLogged

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class EKFYawResetLogged(TestGroup):
    def ekf_yaw_reset_logged_for_ekf_type(self, active_type):
        '''fly an in-filter yaw reset with the given EKF active'''
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
        self.arm_vehicle()
        self.user_takeoff(alt_min=5)

        # the mis-oriented compass causes an in-filter emergency
        # yaw reset shortly after takeoff:
        self.wait_statustext("EKF%u IMU. emergency yaw reset" % active_type,
                             timeout=10, regex=True, check_context=True)
        self.delay_sim_time(5, reason="let the event reach the log")
        self.do_RTL()

        # the active EKF's internal yaw reset must be noticed by
        # AHRS, which logs an EKF_YAW_RESET event:
        dfreader = self.dfreader_for_current_onboard_log()
        armed = False
        while True:
            m = dfreader.recv_match(type='EV')
            if m is None:
                raise NotAchievedException("Did not find EKF_YAW_RESET event after arming")
            if m.Id == 10:  # LogEvent::ARMED
                armed = True
            if armed and m.Id == 62:  # LogEvent::EKF_YAW_RESET
                break
        self.progress("Found EKF_YAW_RESET event in log")

        self.context_pop()

    @testcase(bucket="CopterTests2a")
    def EKFYawResetLogged(self):
        '''in-filter EKF yaw resets must propagate through AHRS and be logged'''
        # the emergency yaw reset request is routed by AHRS to the
        # active estimator only, so fly once with each filter active;
        # the other filter remains enabled throughout.  Each iteration
        # gets its own context so that its statustext collection does
        # not see the previous iteration's messages.
        for active_type in 2, 3:
            self.start_subtest("active EKF type %u" % active_type)
            self.ekf_yaw_reset_logged_for_ekf_type(active_type)
