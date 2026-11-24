'''
Copter autotest: TakeoffGroundEffectAlt

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TakeoffGroundEffectAlt(TestGroup):
    @testcase(bucket="CopterTests2d")
    def TakeoffGroundEffectAlt(self):
        '''Test GNDEFF_ALT and GNDEFF_TMO gate the ground-effect compensation window'''
        # SIM_BARO_GEFF_M injects a real baro static-pressure error near the
        # ground so the compensation window has something to compensate for;
        # without it the detector parameters would be exercised but the
        # underlying baro error they exist to mitigate wouldn't be present.
        self.set_parameters({
            "LOG_FILE_DSRMROT": 1,
            "SIM_BARO_GEFF_M": 1.0,
        })
        self.progress("Making sure we'll have a short log to look at")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()

        # Subtest A: large threshold - takeoff_expected persists at 5m
        self.start_subtest("Large GNDEFF_ALT keeps ground effect at 5m")
        self.set_parameter("GNDEFF_ALT", 10)
        self.takeoff(5, mode='ALT_HOLD')
        self.delay_sim_time(5, reason='let the takeoff window expire before landing')
        self.change_mode('LAND')
        self.wait_disarmed()
        durations_large = self.get_takeoffexpected_durations_from_current_onboard_log(ignore_multi=True)
        total_large = sum(durations_large)
        self.progress("takeoff_expected total with GNDEFF_ALT=10: %fs" % total_large)
        if total_large < 3:
            raise NotAchievedException(
                "takeoff_expected should persist with large threshold (got %fs, want>3)" % total_large)

        # Subtest B: small threshold - takeoff_expected clears quickly
        # GNDEFF_TMO=0 so only the altitude check releases the window,
        # giving subtest C a baseline without the default minimum hold.
        self.start_subtest("Small GNDEFF_ALT clears ground effect at 5m")
        self.set_parameters({
            "GNDEFF_ALT": 0.5,
            "GNDEFF_TMO": 0,
        })
        self.takeoff(5, mode='ALT_HOLD')
        self.delay_sim_time(5, reason='let the takeoff window expire before landing')
        self.change_mode('LAND')
        self.wait_disarmed()
        durations_small = self.get_takeoffexpected_durations_from_current_onboard_log(ignore_multi=True)
        total_small = sum(durations_small)
        self.progress("takeoff_expected total with GNDEFF_ALT=0.5: %fs" % total_small)

        # Comparative assertion: large threshold should have longer duration
        if total_small >= total_large:
            raise NotAchievedException(
                "Smaller threshold should have shorter ground effect (small=%fs >= large=%fs)"
                % (total_small, total_large))

        # Subtest C: GNDEFF_TMO requires both timeout AND altitude
        # With small altitude threshold but timeout set, ground effect should persist longer
        self.start_subtest("GNDEFF_TMO extends ground effect duration")
        self.set_parameters({
            "GNDEFF_ALT": 0.5,  # Small threshold - would clear quickly without timeout
            "GNDEFF_TMO": 3,    # Require 3s timeout as well
        })
        self.takeoff(5, mode='ALT_HOLD')
        self.delay_sim_time(5, reason='let the takeoff window expire before landing')
        self.change_mode('LAND')
        self.wait_disarmed()
        durations_tmo = self.get_takeoffexpected_durations_from_current_onboard_log(ignore_multi=True)
        total_tmo = sum(durations_tmo)
        self.progress("takeoff_expected total with GNDEFF_TMO=3: %fs" % total_tmo)

        # With timeout, ground effect should persist longer than without (even with small alt threshold)
        if total_tmo <= total_small:
            raise NotAchievedException(
                "GNDEFF_TMO should extend ground effect (tmo=%fs <= no_tmo=%fs)"
                % (total_tmo, total_small))

        # Subtest D: rangefinder HAGL path. On the ground the EKF HAGL reads
        # the rangefinder ground clearance rather than zero, so with GNDCLR
        # above GNDEFF_ALT the windows must be measured from the on-ground
        # reading or takeoff releases before liftoff and touchdown never fires.
        self.start_subtest("Rangefinder HAGL is measured from the on-ground reading")
        self.set_parameters({
            "GNDEFF_ALT": 0.5,
            "GNDEFF_TMO": 0,
            "RNGFND1_TYPE": 100,  # SITL
            "RNGFND1_GNDCLR": 0.6,
        })
        self.reboot_sitl()
        self.takeoff(5, mode='ALT_HOLD')
        self.delay_sim_time(5, reason='let the takeoff window expire before landing')
        self.change_mode('LAND')
        self.wait_disarmed()
        durations_rf = self.get_takeoffexpected_durations_from_current_onboard_log(ignore_multi=True)
        total_rf = sum(durations_rf)
        touchdown_rf = sum(self.get_touchdownexpected_durations_from_current_onboard_log(ignore_multi=True))
        self.progress("takeoff_expected total with rangefinder GNDCLR=0.6: %fs (touchdown %fs)" %
                      (total_rf, touchdown_rf))
        if total_rf < 0.5 * total_small:
            raise NotAchievedException(
                "Rangefinder should not release takeoff early (rf=%fs < half of baro=%fs)"
                % (total_rf, total_small))
        if touchdown_rf <= 0:
            raise NotAchievedException("Rangefinder touchdown_expected never fired")

        # we are not at the home location - reboot so the next test starts there
        self.set_parameter("RNGFND1_TYPE", 0)
        self.reboot_sitl()
