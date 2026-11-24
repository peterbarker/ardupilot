'''
Copter autotest: AHRSSwitchBackendPositionNEReset

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import PreconditionFailedException

from test_discovery import TestGroup
from test_discovery import testcase


class AHRSSwitchBackendPositionNEReset(TestGroup):
    @testcase(bucket="CopterTests1a")
    def AHRSSwitchBackendPositionNEReset(self):
        '''vehicle must not lurch when the active AHRS estimator is changed with divergent NE positions'''
        # glitch the GPS; EKF3 eventually adopts the glitched position
        # while the SIM backend continues to report truth.  Switching
        # estimators then implies a large NE position reset which must
        # be handled by the position controller.
        self.set_parameter("FS_EKF_THRESH", 1)  # avoid EKF failsafe while the glitch is being rejected
        self.takeoff(20, mode='GUIDED')

        # hold in LOITER; unlike GUIDED it moves the position target,
        # not the vehicle, when handling an estimate reset:
        self.set_rc(3, 1500)
        self.change_mode('LOITER')

        self.progress("Applying GPS glitch")
        self.set_parameter("SIM_GPS1_GLTCH_X", 0.0003)  # ~33m north

        self.progress("Waiting for EKF3 to adopt the glitched position")
        tstart = self.get_sim_time()
        divergence = 0
        while divergence < 15:
            if self.get_sim_time_cached() - tstart > 120:
                raise PreconditionFailedException(f"estimates did not diverge ({divergence:.1f}m)")
            divergence = self.get_distance(self.get_location('SIMSTATE'), self.get_location())
            self.progress(f"NE divergence={divergence:.1f}m")
        self.delay_sim_time(10, reason="let position controller settle after glitch adoption")

        start_loc = self.get_location('SIMSTATE')
        self.context_collect('STATUSTEXT')
        self.progress("Switching active estimator from EKF3 to SIM")
        self.set_parameter("AHRS_EKF_TYPE", 10)
        self.wait_statustext("AHRS: SIM active", check_context=True, timeout=10)

        # the vehicle must stay physically put; an unhandled reset
        # leaves it permanently displaced by the position controller's
        # error limit:
        moved = 0
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 20:
            moved = self.get_distance(start_loc, self.get_location('SIMSTATE'))
            self.progress(f"moved={moved:.1f}m")
            if moved > 8:
                raise NotAchievedException(f"Vehicle lurched {moved:.1f}m after estimator switch")
        if moved > 3:
            raise NotAchievedException(f"Vehicle displaced {moved:.1f}m by estimator switch")
        self.progress("Vehicle stayed put across estimator switch")

        # remove the glitch and switch back to EKF3 - yet another
        # reset which must be handled.  Flying home on EKF3 also
        # ensures we navigate in the frame home was recorded in:
        self.set_parameter("SIM_GPS1_GLTCH_X", 0)
        self.set_parameter("AHRS_EKF_TYPE", 3)
        self.wait_statustext("AHRS: EKF3 active", check_context=True, timeout=10)

        # wait for EKF3 to shed the glitch so the vehicle agrees with
        # truth about where it is:
        tstart = self.get_sim_time()
        while True:
            residual = self.get_distance(self.get_location('SIMSTATE'), self.get_location())
            self.progress(f"post-switch-back residual={residual:.1f}m")
            if residual < 5:
                break
            if self.get_sim_time_cached() - tstart > 60:
                raise NotAchievedException(f"EKF3 did not shed the GPS glitch (residual={residual:.1f}m)")

        self.do_RTL()
