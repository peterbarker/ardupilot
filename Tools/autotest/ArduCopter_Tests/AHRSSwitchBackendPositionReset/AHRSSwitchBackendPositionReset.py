'''
Copter autotest: AHRSSwitchBackendPositionReset

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import PreconditionFailedException

from test_discovery import TestGroup
from test_discovery import testcase


class AHRSSwitchBackendPositionReset(TestGroup):
    @testcase(bucket="CopterTests2a")
    def AHRSSwitchBackendPositionReset(self):
        '''vehicle must not lurch when the active AHRS estimator is changed in flight'''
        # drift the baro so EKF3's altitude estimate (baro-based by
        # default) diverges from truth, then switch to the SIM backend,
        # which always reports truth.  The jump in the published
        # altitude estimate must be handled as a position reset by the
        # position controller or the vehicle will aggressively
        # "correct" the apparent altitude error.
        self.takeoff(30, mode='GUIDED')

        # hold in LOITER; unlike GUIDED it moves the position target,
        # not the vehicle, when handling an estimate reset:
        self.set_rc(3, 1500)
        self.change_mode('LOITER')

        self.progress("Diverging EKF3 altitude from truth using baro drift")
        self.set_parameter("SIM_BARO_DRIFT", -0.3)
        self.delay_sim_time(75, reason="allow altitude estimates to diverge")
        # baro drift accumulates in the simulator, so zeroing the rate
        # freezes the accumulated offset:
        self.set_parameter("SIM_BARO_DRIFT", 0)
        # the vehicle chases the drifting estimate, so let it settle:
        self.wait_climbrate(-0.1, 0.1, minimum_duration=10, timeout=60)

        truth_alt = self.get_altitude(altitude_source='SIM_STATE.alt')
        estimated_alt = self.get_altitude(altitude_source='GLOBAL_POSITION_INT.alt')
        divergence = abs(truth_alt - estimated_alt)
        self.progress(f"truth_alt={truth_alt:.1f} estimated_alt={estimated_alt:.1f} divergence={divergence:.1f}")
        if divergence < 8:
            raise PreconditionFailedException(f"estimates did not diverge ({divergence:.1f}m)")

        self.context_collect('STATUSTEXT')
        self.progress("Switching active estimator from EKF3 to SIM")
        self.set_parameter("AHRS_EKF_TYPE", 10)
        self.wait_statustext("AHRS: SIM active", check_context=True, timeout=10)

        # the vehicle must stay physically put.  If the switch is not
        # handled as a reset the position controller's error limit
        # rebases the target onto the new estimate, but that still
        # leaves the vehicle permanently displaced by the error limit
        # (~2.5m); a properly-handled reset moves the target instead,
        # so the vehicle does not move at all:
        moved = 0
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 20:
            current_truth_alt = self.get_altitude(altitude_source='SIM_STATE.alt')
            moved = abs(current_truth_alt - truth_alt)
            self.progress(f"truth_alt={current_truth_alt:.1f} moved={moved:.1f}")
            if moved > 5:
                raise NotAchievedException(f"Vehicle lurched {moved:.1f}m after estimator switch")
        if moved > 1.5:
            raise NotAchievedException(f"Vehicle displaced {moved:.1f}m by estimator switch")
        self.progress("Vehicle stayed put across estimator switch")

        self.do_RTL()

        # we have played with SIM_BARO_DRIFT and that causes the
        # estimators to build up state that takes time to decay - so
        # just reboot.
        self.reboot_sitl()
