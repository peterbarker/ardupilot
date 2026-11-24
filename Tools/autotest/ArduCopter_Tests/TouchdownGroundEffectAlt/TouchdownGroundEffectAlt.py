'''
Copter autotest: TouchdownGroundEffectAlt

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TouchdownGroundEffectAlt(TestGroup):
    @testcase(bucket="CopterTests2d")
    def TouchdownGroundEffectAlt(self):
        '''Test GNDEFF_ALT gates the touchdown ground-effect signal'''
        # touchdown_expected fires only when slow horizontal motion AND slow
        # descent AND near-ground (height < GNDEFF_ALT). Exercise the altitude
        # gate by landing twice from the same altitude with different
        # GNDEFF_ALT values: a small threshold should only fire near the
        # ground, a large threshold (>= takeoff altitude) should fire for the
        # whole descent.
        self.set_parameter("LOG_FILE_DSRMROT", 1)
        self.progress("Making sure we'll have a short log to look at")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()

        # Subtest A: small threshold - touchdown_expected only fires near ground
        self.start_subtest("Small GNDEFF_ALT only triggers touchdown near ground")
        self.set_parameter("GNDEFF_ALT", 1.0)
        self.takeoff(3, mode='GUIDED', alt_minimum_duration=2)
        self.change_mode('LAND')
        self.wait_disarmed()
        durations_small = self.get_touchdownexpected_durations_from_current_onboard_log(ignore_multi=True)
        total_small = sum(durations_small)
        self.progress("touchdown_expected total with GNDEFF_ALT=1.0: %fs" % total_small)
        if total_small < 0.5:
            raise NotAchievedException(
                "touchdown_expected should fire near ground (got %fs, want>0.5)" % total_small)

        # Subtest B: large threshold gates touchdown over the full descent
        self.start_subtest("Large GNDEFF_ALT triggers touchdown for whole descent")
        self.set_parameter("GNDEFF_ALT", 5.0)
        self.takeoff(3, mode='GUIDED', alt_minimum_duration=2)
        self.change_mode('LAND')
        self.wait_disarmed()
        durations_large = self.get_touchdownexpected_durations_from_current_onboard_log(ignore_multi=True)
        total_large = sum(durations_large)
        self.progress("touchdown_expected total with GNDEFF_ALT=5.0: %fs" % total_large)

        # Comparative assertion: a higher threshold catches the descent earlier
        # so total touchdown_expected duration must be longer.
        if total_large <= total_small:
            raise NotAchievedException(
                "Larger threshold should have longer touchdown (large=%fs <= small=%fs)"
                % (total_large, total_small))

        # Subtest C: more than 20m from the takeoff point the baro fallback
        # cannot assume flat ground, so the altitude gate is dropped and the
        # whole slow descent counts, as it did before the gate existed.
        self.start_subtest("Far from takeoff the touchdown gate is dropped")
        self.set_parameter("GNDEFF_ALT", 1.0)
        self.takeoff(3, mode='GUIDED', alt_minimum_duration=2)
        self.fly_guided_move_local(30, 0, 3)
        self.change_mode('LAND')
        self.wait_disarmed()
        durations_far = self.get_touchdownexpected_durations_from_current_onboard_log(ignore_multi=True)
        total_far = sum(durations_far)
        self.progress("touchdown_expected total with GNDEFF_ALT=1.0 30m from takeoff: %fs" % total_far)
        if total_far <= total_small:
            raise NotAchievedException(
                "Dropping the gate far from takeoff should lengthen touchdown (far=%fs <= near=%fs)"
                % (total_far, total_small))

        # we are not at the home location - reboot so the next test starts there
        self.reboot_sitl()
