'''
QuadPlane autotest: AHRSSwitchBackendResets

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import PreconditionFailedException

from test_discovery import TestGroup
from test_discovery import testcase


class AHRSSwitchBackendResets(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def AHRSSwitchBackendResets(self):
        '''vehicle must not move or rotate when the active AHRS estimator changes to a divergent backend'''
        # fly on the SIM backend, which reports truth, while DCM's
        # estimates are made to diverge on all axes: a mis-oriented
        # compass corrupts its yaw, a GPS glitch its position and baro
        # drift its altitude.  Neither backend supplies reset
        # timestamps, so before AHRS kept reset counts a switch
        # between them was undetectable and the attitude and position
        # controllers chased their now-stale targets.
        self.set_parameter("AHRS_EKF_TYPE", 10)
        self.reboot_sitl()

        self.takeoff(20, mode='QLOITER')

        self.progress("Diverging DCM estimates from truth on all axes")
        self.set_parameters({
            "COMPASS_ORIENT": 1,        # yaw 45 degrees; corrupts DCM yaw
            "SIM_GPS1_GLTCH_X": 0.0003,  # ~33m north; corrupts DCM position
            "SIM_BARO_DRIFT": -0.3,     # corrupts DCM altitude
        })
        self.delay_sim_time(60, reason="allow DCM estimates to diverge")
        # baro drift accumulates in the simulator, so zeroing the rate
        # freezes the accumulated offset:
        self.set_parameter("SIM_BARO_DRIFT", 0)
        self.delay_sim_time(5, reason="let things settle")

        truth_yaw_deg = math.degrees(self.assert_receive_message('SIMSTATE').yaw)
        truth_loc = self.get_location('SIMSTATE')
        truth_alt = self.get_altitude(altitude_source='SIM_STATE.alt')

        self.context_collect('STATUSTEXT')
        self.progress("Switching active estimator from SIM to DCM")
        self.set_parameter("AHRS_EKF_TYPE", 0)
        self.wait_statustext("AHRS: DCM active", check_context=True, timeout=10)

        # ensure the switch really implied resets on each axis, else
        # this test is vacuous:
        yaw_divergence = self.heading_delta(self.get_heading(), truth_yaw_deg)
        ne_divergence = self.get_distance(self.get_location(), truth_loc)
        alt_divergence = abs(self.get_altitude(altitude_source='GLOBAL_POSITION_INT.alt') - truth_alt)
        self.progress(f"divergences: yaw={yaw_divergence:.1f}deg NE={ne_divergence:.1f}m alt={alt_divergence:.1f}m")
        if yaw_divergence < 20:
            raise PreconditionFailedException(f"yaw estimates did not diverge ({yaw_divergence:.1f}deg)")
        if ne_divergence < 15:
            raise PreconditionFailedException(f"NE estimates did not diverge ({ne_divergence:.1f}m)")
        if alt_divergence < 8:
            raise PreconditionFailedException(f"altitude estimates did not diverge ({alt_divergence:.1f}m)")

        # the vehicle must stay physically put on all axes; unhandled
        # resets rotate the vehicle to chase its stale yaw target and
        # leave it displaced by the position controller error limits:
        rotated = 0
        moved_ne = 0
        moved_alt = 0
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 20:
            rotated = self.heading_delta(math.degrees(self.assert_receive_message('SIMSTATE').yaw), truth_yaw_deg)
            moved_ne = self.get_distance(truth_loc, self.get_location('SIMSTATE'))
            moved_alt = abs(self.get_altitude(altitude_source='SIM_STATE.alt') - truth_alt)
            self.progress(f"rotated={rotated:.1f}deg moved_ne={moved_ne:.1f}m moved_alt={moved_alt:.1f}m")
            if rotated > 20:
                raise NotAchievedException(f"Vehicle rotated {rotated:.1f}deg after estimator switch")
            if moved_ne > 8:
                raise NotAchievedException(f"Vehicle lurched {moved_ne:.1f}m horizontally after estimator switch")
            if moved_alt > 5:
                raise NotAchievedException(f"Vehicle lurched {moved_alt:.1f}m vertically after estimator switch")
        if rotated > 10:
            raise NotAchievedException(f"Vehicle heading displaced {rotated:.1f}deg by estimator switch")
        if moved_ne > 3:
            raise NotAchievedException(f"Vehicle displaced {moved_ne:.1f}m horizontally by estimator switch")
        if moved_alt > 1.5:
            raise NotAchievedException(f"Vehicle displaced {moved_alt:.1f}m vertically by estimator switch")
        self.progress("Vehicle stayed put across estimator switch")

        # return to sane estimates before coming home:
        self.set_parameters({
            "COMPASS_ORIENT": 0,
            "SIM_GPS1_GLTCH_X": 0,
            "AHRS_EKF_TYPE": 10,
        })
        self.do_RTL()

        # SIM_BARO_DRIFT is a rate: zeroing it stops the offset it has
        # accumulated from growing but leaves it in place for the life
        # of the SITL process, and no parameter records it, so a
        # context revert cannot undo it.  Recalibrating the barometer
        # moves the ground reference but leaves the EKF to absorb the
        # resulting step over several seconds, during which the next
        # test can arm and take home from a height still being
        # corrected.  Reboot instead: it discards the simulator and
        # filter state together, with no transient to race.
        self.reboot_sitl()
