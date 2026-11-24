'''
Copter autotest: AHRSSwitchBackendYawReset

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import PreconditionFailedException

from test_discovery import TestGroup
from test_discovery import testcase


class AHRSSwitchBackendYawReset(TestGroup):
    @testcase(bucket="CopterTests1d")
    def AHRSSwitchBackendYawReset(self):
        '''vehicle must not spin when the active AHRS estimator is changed with divergent yaws'''
        # fly with a mis-oriented compass and the GSF emergency yaw
        # reset disabled so EKF3's yaw estimate is persistently wrong,
        # while the SIM backend continues to report truth.  Switching
        # estimators then implies a yaw reset which must be handled by
        # the attitude controller or the vehicle physically rotates to
        # chase the apparent heading error.
        self.context_push()
        self.set_parameters({
            "COMPASS_ORIENT": 1,    # yaw 45 degrees
            "COMPASS_USE2": 0,      # disable backup compasses to avoid pre-arm failures
            "COMPASS_USE3": 0,
            "EK3_GSF_USE_MASK": 0,  # do not let the GSF fix the bad yaw
            "FS_EKF_THRESH": 1,     # the bad yaw inflates variances; don't failsafe
        })
        self.reboot_sitl()

        self.takeoff(20, mode='GUIDED')

        # hold in LOITER; poor position hold (toilet-bowling) is
        # expected with this much yaw error:
        self.set_rc(3, 1500)
        self.change_mode('LOITER')
        self.delay_sim_time(10, reason="let the vehicle settle in LOITER")

        estimated_yaw_deg = self.get_heading()
        truth_yaw_deg = math.degrees(self.assert_receive_message('SIMSTATE').yaw)
        divergence = self.heading_delta(estimated_yaw_deg, truth_yaw_deg)
        self.progress(f"truth_yaw={truth_yaw_deg:.1f} estimated_yaw={estimated_yaw_deg:.1f} divergence={divergence:.1f}")
        if divergence < 18:
            raise PreconditionFailedException(f"yaw estimates did not diverge ({divergence:.1f}deg)")

        start_truth_yaw_deg = math.degrees(self.assert_receive_message('SIMSTATE').yaw)
        self.context_collect('STATUSTEXT')
        self.progress("Switching active estimator from EKF3 to SIM")
        self.set_parameter("AHRS_EKF_TYPE", 10)
        self.wait_statustext("AHRS: SIM active", check_context=True, timeout=10)

        # the vehicle must not physically rotate; an unhandled yaw
        # reset makes the attitude controller rotate the vehicle by
        # the divergence to chase its stale yaw target:
        rotated = 0
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 20:
            current_truth_yaw_deg = math.degrees(self.assert_receive_message('SIMSTATE').yaw)
            rotated = self.heading_delta(current_truth_yaw_deg, start_truth_yaw_deg)
            self.progress(f"truth_yaw={current_truth_yaw_deg:.1f} rotated={rotated:.1f}")
            if rotated > 20:
                raise NotAchievedException(f"Vehicle rotated {rotated:.1f}deg after estimator switch")
        if rotated > 10:
            raise NotAchievedException(f"Vehicle heading displaced {rotated:.1f}deg by estimator switch")
        self.progress("Vehicle heading held across estimator switch")

        self.do_RTL()
        self.context_pop()
        self.reboot_sitl()
