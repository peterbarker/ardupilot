'''
Helicopter autotest: PosHoldTakeOff

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PosHoldTakeOff(TestGroup):
    @testcase(bucket="Helicopter")
    def PosHoldTakeOff(self):
        """ensure vehicle stays put until it is ready to fly"""
        self.set_parameter("PILOT_TKO_ALT_M", 7.0)
        self.change_mode('POSHOLD')
        self.zero_throttle()
        self.set_rc(8, 1000)
        self.wait_ready_to_arm()
        # Arm
        self.arm_vehicle()
        self.progress("Raising rotor speed")
        self.set_rc(8, 2000)
        self.progress("wait for rotor runup to complete")
        self.wait_servo_channel_value(8, 1659, timeout=10)
        self.delay_sim_time(20, reason="rotor runup to complete")
        # check we are still on the ground...
        max_relalt = 1  # metres
        relative_alt = self.get_altitude(relative=True)
        if abs(relative_alt) > max_relalt:
            raise NotAchievedException("Took off prematurely (abs(%f)>%f)" %
                                       (relative_alt, max_relalt))

        self.progress("Pushing collective past half-way")
        self.set_rc(3, 1600)
        self.delay_sim_time(0.5, reason="collective input to settle")
        self.hover()

        # make sure we haven't already reached alt:
        relative_alt = self.get_altitude(relative=True)
        max_initial_alt = 1.5  # metres
        if abs(relative_alt) > max_initial_alt:
            raise NotAchievedException("Took off too fast (%f > %f" %
                                       (abs(relative_alt), max_initial_alt))

        self.progress("Monitoring takeoff-to-alt")
        self.wait_altitude(6, 8, relative=True, minimum_duration=5)
        self.progress("takeoff OK")

        self.land_and_disarm()
