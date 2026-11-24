'''
QuadPlane autotest: PrecisionLanding

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PrecisionLanding(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def PrecisionLanding(self):
        '''VTOL precision landing'''

        self.install_applet_script_context("plane_precland.lua")

        self.set_parameters({
            "SCR_ENABLE": 1,
            "PLND_ENABLED": 1,
            "PLND_TYPE": 4,
            "SIM_PLD_ENABLE":   1,
            "SIM_PLD_HEIGHT" : 0,
            "SIM_PLD_ALT_LMT" : 50,
            "SIM_PLD_DIST_LMT" : 30,
            "RNGFND1_TYPE": 100,
            "RNGFND1_PIN" : 0,
            "RNGFND1_SCALING" : 12.2,
            "RNGFND1_MAX" : 50.00,
            "RNGFND_LANDING" : 1,
        })

        self.reboot_sitl()

        self.set_parameters({
            "PLND_ALT_CUTOFF" : 5,
            "SIM_SPEEDUP" : 10,
            })

        self.context_collect('STATUSTEXT')

        self.scripting_restart()
        self.wait_text("PLND: Loaded", check_context=True)

        self.wait_ready_to_arm()

        # place the target near the vehicle's current position - which,
        # having just rebooted, is the spawn position.  Sampling the
        # position before the reboot places the target wherever the
        # previous test happened to leave the vehicle, which can be
        # hundreds of metres from where QRTL will descend:
        here = self.get_location()
        target = self.offset_location_ne(here, 20, 0)
        self.set_parameters({
            "SIM_PLD_LAT": target.lat,
            "SIM_PLD_LON": target.lng,
        })

        self.change_mode("GUIDED")
        self.arm_vehicle()
        self.takeoff(60, 'GUIDED')
        self.wait_altitude(58, 62, relative=True)
        self.drain_mav()
        self.change_mode("QRTL")

        self.wait_text("PLND: Target Acquired", check_context=True, timeout=60)

        self.wait_disarmed(timeout=180)
        loc2 = self.get_location()
        error = self.get_distance(target, loc2)
        self.progress("Target error %.1fm" % error)
        if error > 2:
            raise NotAchievedException("too far from target %.1fm" % error)
