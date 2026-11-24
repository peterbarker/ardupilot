'''
Copter autotest: UBloxNavSolInSavedConfig

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class UBloxNavSolInSavedConfig(TestGroup):
    def ublox_sim_set_saved_nav_sol_rate(self, rate):
        '''set the NAV-SOL rate in the simulated u-blox's saved
        configuration, as another tool (e.g. u-center) might'''
        filepath = self.ublox_sim_config_filepath()
        with open(filepath, "rb") as f:
            data = bytearray(f.read())
        data[self.UBX_SIM_NAV_SOL_RATE_OFFSET] = rate
        with open(filepath, "wb") as f:
            f.write(data)

    @testcase(bucket="CopterTests2d")
    def UBloxNavSolInSavedConfig(self):
        '''check NAV-SOL enabled in a u-blox's saved configuration is turned off by the next save'''
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "GPS_AUTO_CONFIG": 1,
            "GPS_SAVE_CFG": 2,
            "SIM_GPS1_UBXMDL": self.UBX_MODEL_M8N_SPG301,
        })
        self.ublox_sim_reset_saved_config()
        self.ublox_sim_boot()

        self.start_subtest("NAV-SOL enabled in saved configuration alone doesn't cause a save")
        self.ublox_sim_set_saved_nav_sol_rate(1)
        saves = self.ublox_sim_count_saves(self.ublox_sim_boot())
        if saves != 0:
            raise NotAchievedException("u-blox configuration saved %u times on boot" % saves)
        if self.ublox_sim_saved_nav_sol_rate() != 1:
            raise NotAchievedException("Saved NAV-SOL rate changed without a save")

        self.start_subtest("A save for another reason turns NAV-SOL off")
        self.set_parameter("GPS_NAVFILTER", 7)
        saves = self.ublox_sim_count_saves(self.ublox_sim_boot())
        if saves != 1:
            raise NotAchievedException("Expected one u-blox configuration save, got %u" % saves)
        if self.ublox_sim_saved_nav_sol_rate() != 0:
            raise NotAchievedException("NAV-SOL still enabled in saved configuration")

        self.context_pop()
        self.ublox_sim_reset_saved_config()
        self.reboot_sitl()
