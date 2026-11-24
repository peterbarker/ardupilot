'''
Copter autotest: EK3SrcSwitchPosDownReset

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import PreconditionFailedException

from test_discovery import TestGroup
from test_discovery import testcase


class EK3SrcSwitchPosDownReset(TestGroup):
    @testcase(bucket="CopterTests1e")
    def EK3SrcSwitchPosDownReset(self):
        '''in-filter EKF position-down resets must be handled by the position controller'''
        # EKF3 uses the baro for height by default; drift the baro,
        # then switch EKF3's height source to GPS mid-flight.  EKF3
        # performs an in-filter position-down reset (ResetPositionD)
        # which must flow through the AHRS reset counters to the
        # position controller.
        self.takeoff(30, mode='GUIDED')
        self.set_rc(3, 1500)
        self.change_mode('LOITER')

        self.progress("Diverging EKF3 altitude from truth using baro drift")
        self.set_parameter("SIM_BARO_DRIFT", -0.3)
        self.delay_sim_time(75, reason="allow altitude estimates to diverge")
        self.set_parameter("SIM_BARO_DRIFT", 0)
        self.wait_climbrate(-0.1, 0.1, minimum_duration=10, timeout=60)

        truth_alt = self.get_altitude(altitude_source='SIM_STATE.alt')
        estimated_alt = self.get_altitude(altitude_source='GLOBAL_POSITION_INT.alt')
        divergence = abs(truth_alt - estimated_alt)
        self.progress(f"truth_alt={truth_alt:.1f} estimated_alt={estimated_alt:.1f} divergence={divergence:.1f}")
        if divergence < 8:
            raise PreconditionFailedException(f"estimates did not diverge ({divergence:.1f}m)")

        self.progress("Switching EKF3 height source to GPS")
        self.set_parameter("EK3_SRC1_POSZ", 3)

        # EKF3 resets its height estimate to the GPS altitude; the
        # vehicle must stay physically put:
        moved = 0
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 20:
            current_truth_alt = self.get_altitude(altitude_source='SIM_STATE.alt')
            moved = abs(current_truth_alt - truth_alt)
            self.progress(f"truth_alt={current_truth_alt:.1f} moved={moved:.1f}")
            if moved > 5:
                raise NotAchievedException(f"Vehicle lurched {moved:.1f}m after height source switch")
        if moved > 1.5:
            raise NotAchievedException(f"Vehicle displaced {moved:.1f}m by height source switch")

        # confirm the EKF actually adopted the GPS height, else the
        # test is vacuous:
        estimated_alt = self.get_altitude(altitude_source='GLOBAL_POSITION_INT.alt')
        current_truth_alt = self.get_altitude(altitude_source='SIM_STATE.alt')
        residual = abs(current_truth_alt - estimated_alt)
        self.progress(f"post-switch estimate residual={residual:.1f}m")
        if residual > 5:
            raise NotAchievedException(f"EKF3 did not adopt GPS height (residual {residual:.1f}m)")
        self.progress("In-filter height reset handled cleanly")

        self.do_RTL()

        # we have played with SIM_BARO_DRIFT and that causes the
        # estimators to build up state that takes time to decay - so
        # just reboot.
        self.reboot_sitl()
