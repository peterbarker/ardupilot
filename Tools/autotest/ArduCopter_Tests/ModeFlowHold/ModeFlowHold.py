'''
Copter autotest: ModeFlowHold

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ModeFlowHold(TestGroup):
    @testcase(bucket="CopterTests1c")
    def ModeFlowHold(self):
        '''test FlowHold mode - position hold and flow-based height estimation'''
        self.set_parameters({
            "SIM_FLOW_ENABLE": 1,
            "FLOW_TYPE": 10,
            # the height estimator discards negative instantaneous
            # heights, so flow noise biases its estimate low; this test
            # is about the estimator arithmetic, not noise rejection:
            "SIM_FLOW_RND": 0,
            # a little wind so position-hold is against an external
            # force rather than just coming to rest:
            "SIM_WIND_SPD": 1,
            "SIM_WIND_DIR": 225,
            "SIM_WIND_T": 1,  # full wind at low altitude (no shear)
        })
        self.reboot_sitl()

        # ground truth for height-above-ground comes from the simulated
        # GPS; the EKF height (and anything derived from it, e.g.
        # GLOBAL_POSITION_INT.relative_alt) is corrupted by baro drift
        # later in this test
        def true_agl_m(ground_alt_m):
            m = self.assert_receive_message('GPS_RAW_INT')
            return m.alt * 0.001 - ground_alt_m

        self.wait_ready_to_arm()
        ground_alt_m = self.assert_receive_message('GPS_RAW_INT').alt * 0.001

        self.takeoff(8, mode='FLOWHOLD')

        self.start_subtest("hold position after pilot input is released")
        # flow is only used from 3s after arming:
        self.delay_sim_time(5, "let FlowHold settle")
        self.set_rc(2, 1200)
        self.wait_groundspeed(1.0, 100, timeout=10)
        self.set_rc(2, 1500)
        self.wait_groundspeed(0, 0.3, timeout=30, minimum_duration=5)
        loc = self.get_location()
        self.delay_sim_time(15, "watch for drift")
        drift_m = self.get_distance(loc, self.get_location())
        self.progress("Drifted %.2fm while holding" % drift_m)
        if drift_m > 3:
            raise NotAchievedException("Drifted %.2fm in FlowHold" % drift_m)

        self.start_subtest("height estimate recovers from EKF height error")
        # FlowHold scales flow to a velocity using its own height
        # estimate, broadcast as named float HEST.  Check it currently
        # agrees with the true height:
        hest_m = self.assert_receive_named_value_float('HEST').value
        agl_m = true_agl_m(ground_alt_m)
        self.progress("HEST %.2fm true-AGL %.2fm" % (hest_m, agl_m))
        if abs(hest_m - agl_m) > 1.5:
            raise NotAchievedException(
                "HEST %.2fm does not match true height %.2fm" %
                (hest_m, agl_m))

        # EK3's default height source is the baro.  Drift the baro low;
        # the EKF height sinks with it and the height controller climbs
        # the vehicle to hold its altitude target, leaving the vehicle
        # higher above the ground than FlowHold's height estimate.
        self.progress("Drifting baro to give EKF an incorrect height")
        self.set_parameter("SIM_BARO_DRIFT", -0.35)
        want_agl_m = 10
        tstart = self.get_sim_time()
        while true_agl_m(ground_alt_m) < want_agl_m:
            if self.get_sim_time_cached() - tstart > 60:
                raise NotAchievedException("Did not climb with baro drift")
        self.set_parameter("SIM_BARO_DRIFT", 0)

        # the height estimator only updates when it sees significant
        # delta-velocity and delta-flow; at this height that takes hard
        # accelerations, so bang the roll and pitch sticks back and
        # forth (out of phase) while waiting for the estimate to
        # converge on the true height
        self.progress("Stirring sticks to excite the height estimator")
        tstart = self.get_sim_time()
        last_stick_flip = 0
        last_report = 0
        flip_pitch = True
        rc_pitch = 2000
        rc_roll = 2000
        try:
            while True:
                now = self.get_sim_time_cached()
                if now - tstart > 150:
                    raise NotAchievedException(
                        "HEST did not converge; HEST %.2fm true %.2fm" %
                        (hest_m, agl_m))
                if now - last_stick_flip > 0.5:
                    if flip_pitch:
                        rc_pitch = 3000 - rc_pitch
                        self.set_rc(2, rc_pitch)
                    else:
                        rc_roll = 3000 - rc_roll
                        self.set_rc(1, rc_roll)
                    flip_pitch = not flip_pitch
                    last_stick_flip = now
                m = self.assert_receive_message('NAMED_VALUE_FLOAT')
                if m.name != 'HEST':
                    continue
                hest_m = m.value
                agl_m = true_agl_m(ground_alt_m)
                if now - last_report > 5:
                    self.progress("HEST %.2fm true-AGL %.2fm" % (hest_m, agl_m))
                    last_report = now
                if abs(hest_m - agl_m) < 0.4:
                    self.progress(
                        "HEST converged in %.1fs; HEST %.2fm true %.2fm" %
                        (now - tstart, hest_m, agl_m))
                    break
        finally:
            self.set_rc_from_map({1: 1500, 2: 1500})

        self.do_RTL()

        # we have played with SIM_BARO_DRIFT and that causes the
        # estimators to build up state that takes time to decay - so
        # just reboot.
        self.reboot_sitl()
