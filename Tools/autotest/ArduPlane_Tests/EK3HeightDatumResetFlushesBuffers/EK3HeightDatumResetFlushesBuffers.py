'''
Plane autotest: EK3HeightDatumResetFlushesBuffers

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class EK3HeightDatumResetFlushesBuffers(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def EK3HeightDatumResetFlushesBuffers(self):
        '''Verify resetHeightDatum flushes baro and output observer buffers'''
        # Plane::update_home() runs every 5 s while disarmed (with
        # GPS lock) and calls ahrs.resetHeightDatum() after a baro
        # recalibration.  Without the buffer-flush fix, stale pre-
        # recalibration baro samples in storedBaro continue to be
        # fused as delayed observations after the reset, producing
        # a brief altitude transient (~0.3-0.5 m).  With the fix
        # storedBaro and the output observer buffers are cleared
        # so reported altitude stays near zero post-reset.
        #
        # The default 5 s update_home cadence does not let drift
        # accumulate enough to make the transient unambiguous, so
        # we suspend periodic resets (HOME_RESET_ALT=-1), inject
        # drift, then re-enable to fire one observable reset.
        #
        # resetHeightDatum is a no-op while !onGround in the EKF, so
        # the test must start from a known on-ground state.  Earlier
        # tests in PlaneTests1b can leave the simulated aircraft
        # airborne (e.g. mid-descent), in which case Plane's
        # barometer.update_calibration() would still run while the
        # EKF reset was skipped, producing a baro-vs-EKF discontinuity
        # that the EKF then chases via the slow baroHgtOffset filter
        # -- exactly the post-reset transient this test is meant to
        # rule out.  Reboot so the SITL aircraft is reliably on the
        # ground at the start of the test.
        self.reboot_sitl()
        # this test never arms, so its boot only produces an onboard
        # log if disarmed logging is enabled.  LOG_DISARMED=1 is only a
        # suite *default*, and predecessors which restart SITL with
        # their own defaults can leave it 0 - in which case this boot
        # has no log and the scan below silently reads the previous
        # boot's log
        self.set_parameter("LOG_DISARMED", 1)
        self.wait_ready_to_arm()

        # suspend periodic resets so baro drift can accumulate
        self.set_parameter("HOME_RESET_ALT", -1)
        self.delay_sim_time(2, reason="baro drift to accumulate")

        # populate storedBaro with samples offset from where the
        # post-recalibration ground reference will sit.  4 s of
        # 0.5 m/s drift offsets the buffer by ~2 m -- well above
        # natural noise so any post-reset transient is unambiguous.
        drift_start_us = int(self.get_sim_time() * 1.0e6)
        self.set_parameter("SIM_BARO_DRIFT", 0.5)
        self.delay_sim_time(4, reason="baro buffer to fill with drift")
        self.set_parameter("SIM_BARO_DRIFT", 0)

        # re-enable periodic resets; the next 5 s tick fires
        # update_home -> resetHeightDatum
        self.set_parameter("HOME_RESET_ALT", 0)
        self.delay_sim_time(8, reason="home reset to trigger")

        # find the reset in the dataflash log: PD jumps by >0.5 m
        # between adjacent samples when resetHeightDatum fires
        # (drift makes PD change at ~0.5 m/s, far slower than
        # one sample interval).  Then measure peak |PD| in the
        # 2 s window post-reset.
        dfreader = self.dfreader_for_current_onboard_log()
        last_pd = None
        reset_us = None
        seen_test_window = False
        peak_excursion = 0.0
        while True:
            m = dfreader.recv_match(type='XKF1', condition='XKF1.C==0')
            if m is None:
                break
            if m.TimeUS < drift_start_us:
                continue
            seen_test_window = True
            if reset_us is None:
                if last_pd is not None and abs(m.PD - last_pd) > 0.5:
                    reset_us = m.TimeUS
                    self.progress("Reset detected at %.2f s (PD %.3f -> %.3f)" %
                                  (m.TimeUS / 1.0e6, last_pd, m.PD))
                last_pd = m.PD
                continue
            if m.TimeUS - reset_us > 2.0e6:
                break
            peak_excursion = max(peak_excursion, abs(m.PD))

        if not seen_test_window:
            raise NotAchievedException(
                "Onboard log contains no data from this test's "
                "timeframe - scanned a previous boot's log?")

        if reset_us is None:
            raise NotAchievedException(
                "Did not detect resetHeightDatum in log "
                "(was update_home triggered?)")

        self.progress("Peak |XKF1.PD| over 2 s post-reset: %.3f m" %
                      peak_excursion)
        if peak_excursion > 0.1:
            raise NotAchievedException(
                "Post-reset altitude transient exceeds 0.1 m: %.3f m "
                "(stale baro buffer not flushed on resetHeightDatum)" %
                peak_excursion)

        # we have played with SIM_BARO_DRIFT and that causes the
        # estimators to build up state that takes time to decay - so
        # just reboot.
        self.reboot_sitl()
