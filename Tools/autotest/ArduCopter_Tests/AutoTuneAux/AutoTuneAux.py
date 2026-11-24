'''
Copter autotest: AutoTuneAux

AP_FLAKE8_CLEAN
'''

import copy

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AutoTuneAux(TestGroup):
    @testcase(bucket="CopterTests2a")
    def AutoTuneAux(self):
        """Test autotune with gains being tested using the Aux function"""

        # autotune changes a set of parameters on the vehicle which
        # are not in our context.  That changes the flight
        # characteristics, which we can't afford between runs.  So
        # completely reset the simulated vehicle after the run is
        # complete by "customising" the commandline here:
        self.customise_SITL_commandline([])

        self.set_parameters({
            "RC8_OPTION": 180,
            "AUTOTUNE_AGGR": 0.1,
            "AUTOTUNE_MIN_D": 0.0004,
        })

        self.takeoff(10, mode='LOITER')

        def print_gains(name, gains):
            self.progress(f"AUTOTUNE {name} gains are P:%f I:%f D:%f" % (
                gains["ATC_RAT_RLL_P"],
                gains["ATC_RAT_RLL_I"],
                gains["ATC_RAT_RLL_D"]
            ))

        def get_roll_gains(name):
            ret = self.get_parameters([
                "ATC_RAT_RLL_D",
                "ATC_RAT_RLL_I",
                "ATC_RAT_RLL_P",
            ], verbose=False)
            print_gains(name, ret)
            return ret

        def gains_same(gains1, gains2):
            for c in 'P', 'I', 'D':
                p_name = f"ATC_RAT_RLL_{c}"
                if abs(gains1[p_name] - gains2[p_name]) > 0.00001:
                    return False
            return True

        self.progress("Take a copy of original gains")
        original_gains = get_roll_gains("pre-tuning")
        scaled_original_gains = copy.copy(original_gains)
        scaled_original_gains["ATC_RAT_RLL_I"] *= 0.1

        pre_rllt = self.get_parameter("ATC_RAT_RLL_FLTT")

        # hold position in loiter and run autotune
        self.change_mode('AUTOTUNE')

        tstart = self.get_sim_time()
        sim_time_expected = 5000
        deadline = tstart + sim_time_expected
        while self.get_sim_time_cached() < deadline:
            now = self.get_sim_time_cached()
            m = self.mav.recv_match(type='STATUSTEXT',
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue
            self.progress("STATUSTEXT (%u<%u): %s" % (now, deadline, m.text))
            if "Determination Failed" in m.text:
                break
            if "AutoTune: Success" in m.text:
                self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
                post_gains = get_roll_gains("post")
                self.progress("Check original gains are used after tuning finished")
                if not gains_same(original_gains, post_gains):
                    raise NotAchievedException("AUTOTUNE original gains not restored")

                self.change_mode('LOITER')
                self.wait_mode('LOITER')

                self.progress("Use autotuned gains")
                self.set_rc(8, 1850)
                self.delay_sim_time(1, reason="gains to update after switch")
                tuned_gains = get_roll_gains("tuned")
                if gains_same(tuned_gains, original_gains):
                    raise NotAchievedException("AUTOTUNE tuned gains same as pre gains")
                if gains_same(tuned_gains, scaled_original_gains):
                    raise NotAchievedException("AUTOTUNE tuned gains same as scaled pre gains")

                self.progress("Check original gains are re-instated by switch")
                self.set_rc(8, 1100)
                self.delay_sim_time(1, reason="gains to update after switch")
                current_gains = get_roll_gains("set-original")
                if not gains_same(original_gains, current_gains):
                    raise NotAchievedException("AUTOTUNE original gains not restored")

                self.progress("land using Autotune Gains")
                self.set_rc(8, 1850)
                self.delay_sim_time(1, reason="gains to update after switch")
                self.set_rc(3, 1000)
                self.wait_altitude(-1, 5, relative=True)
                self.wait_disarmed()
                self.progress("Check gains are still there after disarm")
                disarmed_gains = get_roll_gains("post-disarm")
                if not gains_same(tuned_gains, disarmed_gains):
                    raise NotAchievedException("AUTOTUNE gains not present on disarm")

                self.reboot_sitl()
                self.progress("Check gains are still there after reboot")
                reboot_gains = get_roll_gains("post-reboot")
                if not gains_same(tuned_gains, reboot_gains):
                    raise NotAchievedException("AUTOTUNE gains not present on reboot")
                self.progress("Check FLTT is unchanged")
                if pre_rllt != self.get_parameter("ATC_RAT_RLL_FLTT"):
                    raise NotAchievedException("AUTOTUNE FLTT was modified")
                return

        raise NotAchievedException("AUTOTUNE failed (%u seconds)" %
                                   (self.get_sim_time() - tstart))
