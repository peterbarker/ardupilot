'''
Helicopter autotest: AutoTune

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AutoTune(TestGroup):
    @testcase(bucket="Helicopter")
    def AutoTune(self):
        """Test autotune mode"""
        # test roll and pitch FF tuning
        self.set_parameters({
            "ATC_ANG_RLL_P": 4.5,
            "ATC_RAT_RLL_P": 0,
            "ATC_RAT_RLL_I": 0.1,
            "ATC_RAT_RLL_D": 0,
            "ATC_RAT_RLL_FF": 0.15,
            "ATC_ANG_PIT_P": 4.5,
            "ATC_RAT_PIT_P": 0,
            "ATC_RAT_PIT_I": 0.1,
            "ATC_RAT_PIT_D": 0,
            "ATC_RAT_PIT_FF": 0.15,
            "ATC_ANG_YAW_P": 4.5,
            "ATC_RAT_YAW_P": 0.18,
            "ATC_RAT_YAW_I": 0.024,
            "ATC_RAT_YAW_D": 0.003,
            "ATC_RAT_YAW_FF": 0.024,
            "AUTOTUNE_AXES": 3,
            "AUTOTUNE_SEQ": 1,
            })

        # Conduct testing from althold
        self.takeoff(10, mode="ALT_HOLD")

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        self.wait_statustext('AutoTune: Success', timeout=1000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.autotune_land_and_save_gains()

        # test pitch rate P and Rate D tuning
        self.set_parameters({
            "AUTOTUNE_AXES": 2,
            "AUTOTUNE_SEQ": 2,
            "AUTOTUNE_GN_MAX": 1.8,
            })

        # Conduct testing from althold
        self.takeoff(10, mode="ALT_HOLD")

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        self.wait_statustext('AutoTune: Success', timeout=1000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.autotune_land_and_save_gains()

        # test Roll rate P and Rate D tuning
        self.set_parameters({
            "AUTOTUNE_AXES": 1,
            "AUTOTUNE_SEQ": 2,
            "AUTOTUNE_GN_MAX": 1.6,
            })

        # Conduct testing from althold
        self.takeoff(10, mode="ALT_HOLD")

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        self.wait_statustext('AutoTune: Success', timeout=1000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.autotune_land_and_save_gains()

        # test Roll and pitch angle P tuning
        self.set_parameters({
            "AUTOTUNE_AXES": 3,
            "AUTOTUNE_SEQ": 4,
            "AUTOTUNE_FRQ_MIN": 5,
            "AUTOTUNE_FRQ_MAX": 50,
            "AUTOTUNE_GN_MAX": 1.6,
            })

        # Conduct testing from althold
        self.takeoff(10, mode="ALT_HOLD")

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        self.wait_statustext('AutoTune: Success', timeout=1000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.autotune_land_and_save_gains()

        # test yaw FF and rate P and Rate D
        self.set_parameters({
            "AUTOTUNE_AXES": 4,
            "AUTOTUNE_SEQ": 3,
            "AUTOTUNE_FRQ_MIN": 10,
            "AUTOTUNE_FRQ_MAX": 70,
            "AUTOTUNE_GN_MAX": 1.4,
            })

        # Conduct testing from althold
        self.takeoff(10, mode="ALT_HOLD")

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        self.wait_statustext('AutoTune: Success', timeout=1000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.autotune_land_and_save_gains()

        # test yaw angle P tuning
        self.set_parameters({
            "AUTOTUNE_AXES": 4,
            "AUTOTUNE_SEQ": 4,
            "AUTOTUNE_FRQ_MIN": 5,
            "AUTOTUNE_FRQ_MAX": 50,
            "AUTOTUNE_GN_MAX": 1.5,
            })

        # Conduct testing from althold
        self.takeoff(10, mode="ALT_HOLD")

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        self.wait_statustext('AutoTune: Success', timeout=1000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.autotune_land_and_save_gains()

        # tune check
        self.set_parameters({
            "AUTOTUNE_AXES": 7,
            "AUTOTUNE_SEQ": 16,
            "AUTOTUNE_FRQ_MIN": 10,
            "AUTOTUNE_FRQ_MAX": 80,
            })

        # Conduct testing from althold
        self.takeoff(10, mode="ALT_HOLD")

        # hold position in loiter
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        self.wait_statustext('AutoTune: Success', timeout=1000)
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.land_and_disarm()

    def autotune_land_and_save_gains(self):
        self.set_rc(3, 1000)
        self.context_collect('STATUSTEXT')
        self.wait_statustext(r"SIM Hit ground at ([0-9.]+) m/s",
                             check_context=True,
                             regex=True)
        self.set_rc(8, 1000)
        self.wait_disarmed()
