'''
Copter autotest: UBloxNoSaveEveryBoot

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class UBloxNoSaveEveryBoot(TestGroup):
    def ublox_sim_saved_bds_b1c_ena(self):
        '''return CFG-SIGNAL-BDS_B1C_ENA from the simulated u-blox's saved configuration'''
        with open(self.ublox_sim_config_filepath(), "rb") as f:
            data = f.read()
        return data[self.UBX_SIM_BDS_B1C_ENA_OFFSET]

    @testcase(bucket="CopterTests2d")
    def UBloxNoSaveEveryBoot(self):
        '''check u-blox receivers are not told to save their configuration on every boot'''
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "GPS_AUTO_CONFIG": 1,
            "GPS_SAVE_CFG": 2,
        })
        for (model, name) in [
                (self.UBX_MODEL_UBLOX6, "u-blox 6 FW 7.03"),
                (self.UBX_MODEL_M8N_FW201, "NEO-M8N FW 2.01"),
                (self.UBX_MODEL_M8_ROM301, "M8 ROM CORE 3.01"),
                (self.UBX_MODEL_M8N_SPG301, "NEO-M8N SPG 3.01"),
                (self.UBX_MODEL_M9N_SPG404, "NEO-M9N SPG 4.04"),
                (self.UBX_MODEL_M10_SPG510, "M10 ROM SPG 5.10"),
        ]:
            self.start_subtest(name)
            self.set_parameter("SIM_GPS1_UBXMDL", model)
            self.ublox_sim_reset_saved_config()
            self.progress("Boot with the receiver at factory configuration")
            saves = self.ublox_sim_count_saves(self.ublox_sim_boot())
            if saves != 1:
                raise NotAchievedException("Expected one u-blox configuration save from factory configuration, got %u" % saves)
            nav_sol_rate = self.ublox_sim_saved_nav_sol_rate()
            self.progress("Saved NAV-SOL rate: %u" % nav_sol_rate)
            if model != self.UBX_MODEL_UBLOX6 and nav_sol_rate != 0:
                raise NotAchievedException("NAV-SOL enabled in saved configuration")
            if model == self.UBX_MODEL_M10_SPG510:
                # the M10 is configured through CFG-VALSET; check that
                # reached the saved configuration
                if self.ublox_sim_saved_bds_b1c_ena() != 1:
                    raise NotAchievedException("M10 CFG-VALSET settings not saved")
            for i in range(2):
                self.progress("Boot with the receiver configured (%u)" % i)
                saves = self.ublox_sim_count_saves(self.ublox_sim_boot())
                if saves != 0:
                    raise NotAchievedException("u-blox configuration saved %u times on boot" % saves)
        self.context_pop()
        self.ublox_sim_reset_saved_config()
        self.reboot_sitl()
