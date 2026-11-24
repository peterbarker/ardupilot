'''
QuadPlane autotest: FastInvertedRecovery

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FastInvertedRecovery(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def FastInvertedRecovery(self):
        '''test recovery from inverted flight is fast'''

        self.set_parameters({
            "Q_A_ACC_R_MAX": 200,
            "Q_A_ACC_P_MAX": 200,
            "Q_A_ACC_Y_MAX": 200,
            "Q_A_RATE_R_MAX": 50,
            "Q_A_RATE_P_MAX": 50,
            "Q_A_RATE_Y_MAX": 50,
        })

        self.wait_ready_to_arm()
        self.takeoff(60, mode='GUIDED', timeout=100)

        self.context_collect('STATUSTEXT')
        self.set_rc(3, 1500)
        self.change_mode('CRUISE')
        self.wait_statustext("Transition done", check_context=True)

        self.progress("Go to inverted flight")
        self.run_auxfunc(43, 2)
        self.wait_roll(180, 3, absolute_value=True)
        self.delay_sim_time(10, reason="inverted flight to stabilise")

        initial_altitude = self.get_altitude(relative=True, timeout=2)
        self.change_mode('QHOVER')

        self.wait_roll(0, 3, absolute_value=True)

        recovery_altitude = self.get_altitude(relative=True, timeout=2)
        alt_change = initial_altitude - recovery_altitude

        self.progress("Recovery AltChange %.1fm" % alt_change)

        # stop asking for inverted flight.  Aux function state is not a
        # parameter, so context_pop() does not clear it, and this test
        # does not reboot - without this the next test on this worker
        # starts with inverted flight still commanded.
        self.run_auxfunc(43, 0)

        max_alt_change = 3
        if alt_change > max_alt_change:
            raise NotAchievedException("Recovery AltChange too high %.1f > %.1f" % (alt_change, max_alt_change))
        self.fly_home_land_and_disarm()
