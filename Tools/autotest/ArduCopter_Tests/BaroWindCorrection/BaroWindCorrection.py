'''
Copter autotest: BaroWindCorrection

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class BaroWindCorrection(TestGroup):
    @testcase(bucket="CopterTests1c")
    def BaroWindCorrection(self):
        '''Test wind estimation and baro position error compensation'''
        self.customise_SITL_commandline(
            [],
            **self.callisto_sitl_kwargs()
        )
        wind_spd_truth = 8.0
        wind_dir_truth = 90.0
        self.set_parameters({
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
            "BARO1_WCF_ENABLE": 1.000000,
        })
        self.reboot_sitl()
        self.set_parameters({
            "BARO1_WCF_FWD": -0.300000,
            "BARO1_WCF_BCK": -0.300000,
            "BARO1_WCF_RGT": 0.300000,
            "BARO1_WCF_LFT": 0.300000,
            "BARO1_WCF_UP": 0.300000,
            "BARO1_WCF_DN": 0.300000,
            "SIM_BARO_WCF_FWD": -0.300000,
            "SIM_BARO_WCF_BAK": -0.300000,
            "SIM_BARO_WCF_RGT": 0.300000,
            "SIM_BARO_WCF_LFT": 0.300000,
            "SIM_BARO_WCF_UP": 0.300000,
            "SIM_BARO_WCF_DN": 0.300000,
            "SIM_WIND_DIR": wind_dir_truth,
            "SIM_WIND_SPD": wind_spd_truth,
            "SIM_WIND_T": 1.000000,
        })
        self.reboot_sitl()

        # require_absolute=True infers a GPS is present
        self.wait_ready_to_arm(require_absolute=False)

        self.progress("Climb to 20m in LOITER and yaw spin for 30 seconds")
        self.takeoff(10, mode="LOITER")
        self.set_rc(4, 1400)
        self.delay_sim_time(30, reason="wind estimation via yaw spin")

        # check wind estimates
        m = self.assert_receive_message('WIND')
        speed_error = abs(m.speed - wind_spd_truth)
        angle_error = abs(m.direction - wind_dir_truth)
        if (speed_error > 1.0):
            raise NotAchievedException("Wind speed incorrect - want %f +-1 got %f m/s" % (wind_spd_truth, m.speed))
        if (angle_error > 15.0):
            raise NotAchievedException(
                "Wind direction incorrect - want %f +-15 got %f deg" %
                (wind_dir_truth, m.direction))
        self.progress("Wind estimate is good, now check height variation for 30 seconds")

        # check height stability over another 30 seconds
        z_min = 1E6
        z_max = -1E6
        tstart = self.get_sim_time()
        while (self.get_sim_time() < tstart + 30):
            m = self.assert_receive_message('LOCAL_POSITION_NED')
            if (m.z > z_max):
                z_max = m.z
            if (m.z < z_min):
                z_min = m.z
        if (z_max-z_min > 0.5):
            raise NotAchievedException("Height variation is excessive")
        self.progress("Height variation is good")

        self.set_rc(4, 1500)
        self.land_and_disarm()
