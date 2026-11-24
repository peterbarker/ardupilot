'''
Copter autotest: BaroGroundEffectRangefinderSwitch

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class BaroGroundEffectRangefinderSwitch(TestGroup):
    @testcase(bucket="CopterTests2d")
    def BaroGroundEffectRangefinderSwitch(self):
        '''EKF height survives a takeoff that switches from the range finder to a baro in ground effect'''
        # With EK3_RNG_USE_HGT Copter uses the range finder for height while
        # taking off, and ALT_HOLD's takeoff ends as the vehicle leaves the
        # ground, so the height source goes back to baro there, still in ground
        # effect, where SIM_BARO_GEFF_M makes the baro read low. A baro offset
        # learned then is carried through the switch and kept after climbing out
        # of the effect; resetting the height to that baro at the switch instead
        # drops it by the whole error.
        self.set_parameters({
            "SIM_BARO_GEFF_M": 3.0,
            "SIM_TERRAIN": 0,  # flat ground at home, whatever an earlier test left
            "RNGFND1_TYPE": 100,
            "RNGFND1_MIN": 0,
            "RNGFND1_MAX": 10,
            "EK3_RNG_USE_HGT": 50,
            "PILOT_TKO_ALT_M": 0,
        })
        self.reboot_sitl()
        self.takeoff(10, mode='ALT_HOLD')
        self.delay_sim_time(10, reason="collect samples clear of the ground effect")
        self.change_mode('LAND')
        self.wait_disarmed()

        dfreader = self.dfreader_for_current_onboard_log()
        armed = False
        home_alt = None
        true_alt = None
        climb = []
        high = []
        while True:
            m = dfreader.recv_match(type=['EV', 'MODE', 'SIM', 'XKF1'])
            if m is None:
                break
            mtype = m.get_type()
            if mtype == 'EV':
                if m.Id == 10:  # LogEvent::ARMED
                    armed = True
            elif mtype == 'MODE':
                if armed and m.Mode == 9:  # LAND: the takeoff and hover are the part under test
                    break
            elif mtype == 'SIM':
                if home_alt is None:
                    home_alt = m.Alt
                true_alt = m.Alt - home_alt
            elif m.C == 0 and armed and true_alt is not None:
                err = -m.PD - true_alt
                if true_alt > 5:
                    high.append(err)
                elif len(high) == 0:
                    climb.append(err)
        if len(climb) < 10 or len(high) < 10:
            raise NotAchievedException("insufficient samples (%u up to 5m, %u above it)" % (len(climb), len(high)))
        worst_climb = min(climb)
        mean_high = sum(high) / len(high)
        self.progress("EKF height - truth: worst %+.2f m from arming to 5 m, mean %+.2f m above 5 m"
                      % (worst_climb, mean_high))
        if worst_climb < -1.0:
            raise NotAchievedException("EKF height dropped %.2f m below truth in the takeoff" % -worst_climb)
        if abs(mean_high) > 0.5:
            raise NotAchievedException("EKF height is %+.2f m off truth after climbing out of ground effect" % mean_high)
