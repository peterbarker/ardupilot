'''
Copter autotest: PosHoldTakeOff

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PosHoldTakeOff(TestGroup):
    @testcase(bucket="CopterTests1c")
    def PosHoldTakeOff(self):
        """ensure vehicle stays put until it is ready to fly"""
        self.context_push()

        self.set_parameter("PILOT_TKO_ALT_M", 7.0)
        self.change_mode('POSHOLD')
        self.set_rc(3, 1000)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.delay_sim_time(2, reason="confirming vehicle stays on ground before throttle")
        # check we are still on the ground...
        relative_alt = self.get_altitude(relative=True)
        if relative_alt > 0.1:
            raise NotAchievedException("Took off prematurely")

        self.progress("Pushing throttle up")
        self.set_rc(3, 1710)
        self.delay_sim_time(0.5, reason="throttle input to take effect")
        self.progress("Bringing back to hover throttle")
        self.set_rc(3, 1500)

        # make sure we haven't already reached alt:
        relative_alt = self.get_altitude(relative=True)
        max_initial_alt = 2.0
        if abs(relative_alt) > max_initial_alt:
            raise NotAchievedException("Took off too fast (%f > %f" %
                                       (relative_alt, max_initial_alt))

        self.progress("Monitoring takeoff-to-alt")
        self.wait_altitude(6.9, 8, relative=True, minimum_duration=10)
        self.progress("takeoff OK")

        self.land_and_disarm()
        self.set_rc(8, 1000)

        self.context_pop()
