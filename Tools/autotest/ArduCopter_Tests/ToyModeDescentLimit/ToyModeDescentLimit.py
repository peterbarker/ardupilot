'''
Copter autotest: ToyModeDescentLimit

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ToyModeDescentLimit(TestGroup):
    @testcase(bucket="CopterTests2b")
    def ToyModeDescentLimit(self):
        '''toy mode slows a full-stick descent close to the ground'''
        # TMODE_ENABLE 1 reads the V2450's buttons from channels 5 to
        # 7; these have none of them pressed
        self.set_rc_from_map({
            5: 1500,
            6: 1000,
            7: 1000,
        })
        self.set_parameter("TMODE_ENABLE", 1)
        self.reboot_sitl()
        self.set_parameters({
            "TMODE_MODE1": 2,   # start in ALT_HOLD, and stay there
            "TMODE_MODE2": 2,
            "TMODE_FLAGS": 0,   # no arming or disarming on throttle
            "FLTMODE_CH": 0,    # toy mode chooses the mode
            "PILOT_SPD_DN": 3,
        })
        self.reboot_sitl()
        self.wait_mode('ALT_HOLD')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(3, 2000)
        self.wait_altitude(20, 50, relative=True)
        self.set_rc(3, 1000)

        # well above the slow-down height (5m, ramping to 8m), the full
        # descent rate
        self.wait_climbrate(-3.2, -2.8, minimum_duration=1)
        self.progress("descending at full rate at %.1fm" % self.get_altitude(relative=True))
        if self.get_altitude(relative=True) < 9:
            raise NotAchievedException("didn't reach the full descent rate above 8m")

        # below 5m, throttle is held at 300 or more: a quarter of
        # PILOT_SPD_DN with the default THR_DZ
        self.wait_altitude(0, 4.5, relative=True)
        self.wait_climbrate(-0.9, -0.6, minimum_duration=2, timeout=5)

        self.wait_landed_and_disarmed()
