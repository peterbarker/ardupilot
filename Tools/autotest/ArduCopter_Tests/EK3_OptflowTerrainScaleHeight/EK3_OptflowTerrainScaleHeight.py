'''
Copter autotest: EK3_OptflowTerrainScaleHeight

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class EK3_OptflowTerrainScaleHeight(TestGroup):
    @testcase(bucket="CopterTests1e")
    def EK3_OptflowTerrainScaleHeight(self):
        '''optical flow scale height from the terrain database is right over slopes'''
        # Above the rangefinder range with EK3_OPTIONS bit 2 the optical flow scale
        # height comes from the terrain database. terrain_srtm_alt is measured up from
        # the EKF origin while the position state is down-positive, so where the terrain
        # sits at the origin altitude the two conventions agree and nothing would
        # discriminate. Off the Kalaupapa cliffs the ground falls about 160 m below the
        # origin, where getting it the wrong way round drives the scale height into the
        # on-ground clamp.
        #
        # GPS navigates here, so flow is not fused into velocity and the trajectory does
        # not depend on the scale height: a correct and an inverted build fly the same
        # path. The scale height still sets the predicted flow rate, so the XKF5
        # innovation consistency ratio is what the test reads.
        #
        # What this does NOT prove is that the database rather than the terrain offset
        # state supplied the height. Measured with the option cleared, the frozen
        # terrain state gives a scale height about 3.7x low and a ratio of 3, well
        # inside the gate, against 0 with the option set and 255 with the sign inverted.
        # So a negative leg on this signal would not discriminate, and is not attempted.
        self.install_terrain_handlers_context()
        self.set_parameters({
            "SIM_FLOW_ENABLE": 1,
            "FLOW_TYPE": 10,
            "EK3_IMU_MASK": 1,
            "TERRAIN_ENABLE": 1,
            "EK3_OPTIONS": 1 << 2,   # OptflowMayUseTerrainAlt
            "LOG_FILE_DSRMROT": 1,
        })
        self.set_analog_rangefinder_parameters()
        self.set_parameter("RNGFND1_MAX", 8)
        self.customise_SITL_commandline(["--home", "KalaupapaCliffs"])

        # terrain requests cannot start until the EKF has a location, so let the vehicle
        # reach armable before timing the delivery
        self.wait_ready_to_arm()
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 120:
                raise NotAchievedException("terrain tiles were never delivered")
            report = self.assert_receive_message("TERRAIN_REPORT", timeout=10)
            if report.pending == 0 and report.loaded > 0:
                break

        # 60 m clears the 185.8 m AMSL ridge 50 m north of home by about 40 m. At 40 m
        # the margin is 19.5 m, and dropping to a few metres AGL would put the
        # rangefinder back in range and bypass the branch under test
        self.takeoff(60, mode='GUIDED')
        # gndOffsetValid surfaces as EKF_POS_VERT_AGL; wait for it to go clear so the
        # terrain offset state is not what is supplying the height
        self.wait_ekf_flags(0, mavutil.mavlink.ESTIMATOR_POS_VERT_AGL, timeout=60)

        # the scale height only reaches the innovation through vehicle velocity, so the
        # window that carries the signal is the traverse, not a hover at the end of it
        window_start_us = self.get_sim_time() * 1e6
        self.fly_guided_move_local(400, 0, 60)
        window_end_us = self.get_sim_time() * 1e6

        report = self.assert_receive_message("TERRAIN_REPORT", timeout=10)
        self.progress("true AGL %.2fm over terrain at %.2fm AMSL"
                      % (report.current_height, report.terrain_height))
        if report.current_height < 150:
            raise NotAchievedException(
                "terrain did not fall away enough to test the scale height (%.1fm)"
                % report.current_height)
        self.disarm_vehicle(force=True)

        dfreader = self.dfreader_for_current_onboard_log()
        worst = 0
        count = 0
        while True:
            m = dfreader.recv_match(type="XKF5")
            if m is None:
                break
            if window_start_us <= m.TimeUS <= window_end_us:
                count += 1
                worst = max(worst, m.NI)
        if count == 0:
            raise NotAchievedException("no XKF5 logged over the traverse")
        self.progress("worst flow innovation ratio %u over %u XKF5 samples"
                      % (worst, count))
        # the ratio is logged as 100x, capped at 255. An inverted scale height saturates
        # the cap and flow is rejected outright; a correct one sits near zero
        if worst > 50:
            raise NotAchievedException(
                "flow innovation ratio reached %u, so the scale height is wrong" % worst)
