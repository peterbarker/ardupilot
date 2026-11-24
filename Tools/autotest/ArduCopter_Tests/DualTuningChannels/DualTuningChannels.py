'''
Copter autotest: DualTuningChannels

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class DualTuningChannels(TestGroup):
    @testcase(bucket="CopterTests2a")
    def DualTuningChannels(self):
        '''check tuning channel options work independently'''
        RC6_MIN = 1100    # yes, this is strange!
        RC6_MAX = 1200    # yes, this is strange!
        TUNE_MIN = 0.25   # m/s
        TUNE_MAX = 10.0   # m/s

        RC7_MIN = 1000
        RC7_MAX = 2000
        TUNE2_MIN = 100  # cm/s
        TUNE2_MAX = 500  # cm/s
        self.set_parameters({
            "RC6_OPTION": 219,  # RC6 used for tuning
            "RC6_MIN": RC6_MIN,
            "RC6_MAX": RC6_MAX,
            "TUNE": 60,         # 60 is x/y loiter speed
            "TUNE_MIN": TUNE_MIN,
            "TUNE_MAX": TUNE_MAX,

            "RC7_OPTION": 220,  # RC7 used for second param tuning
            "RC7_MIN": RC7_MIN,
            "RC7_MAX": RC7_MAX,
            "TUNE2": 28,        # PSC_NE_VEL_I
            "TUNE2_MIN":  TUNE2_MIN,
            "TUNE2_MAX":  TUNE2_MAX,
        })

        self.reboot_sitl()

        self.set_rc(6, RC6_MIN)
        self.wait_parameter_value("LOIT_SPEED_MS", TUNE_MIN)

        self.set_rc(6, RC6_MAX)
        self.wait_parameter_value("LOIT_SPEED_MS", TUNE_MAX)
        self.set_rc(6, RC6_MIN)
        self.wait_parameter_value("LOIT_SPEED_MS", TUNE_MIN)

        self.set_rc(6, int((RC6_MIN+RC6_MAX)/2))
        self.delay_sim_time(1, reason="RC6 change to update parameter")
        # note that this check is also used below ("RC6 is unaffected")
        self.assert_parameter_value("LOIT_SPEED_MS", int((TUNE_MIN+TUNE_MAX)/2), epsilon=1)

        self.set_rc(7, RC7_MIN)
        self.wait_parameter_value("PSC_NE_VEL_I", TUNE2_MIN)

        self.set_rc(7, RC7_MAX)
        self.wait_parameter_value("PSC_NE_VEL_I", TUNE2_MAX)

        # make sure RC6 is unaffected:
        self.assert_parameter_value("LOIT_SPEED_MS", int((TUNE_MIN+TUNE_MAX)/2), epsilon=1)

        self.set_rc(7, int((RC7_MIN+RC7_MAX)/2))
        self.delay_sim_time(1, reason="RC7 change to update parameter")
        self.assert_parameter_value("PSC_NE_VEL_I", int((TUNE2_MIN+TUNE2_MAX)/2), epsilon=1)
