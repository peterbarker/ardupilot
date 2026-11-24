'''
Helicopter autotest: StabilizeTakeOff

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class StabilizeTakeOff(TestGroup):
    @testcase(bucket="Helicopter")
    def StabilizeTakeOff(self):
        """Fly stabilize takeoff"""
        self.change_mode('STABILIZE')
        self.set_rc_from_map({3: 1000, 8: 1000})
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(8, 2000)
        self.progress("wait for rotor runup to complete")
        self.wait_servo_channel_value(8, 1659, timeout=10)
        self.delay_sim_time(20, reason="rotor runup to complete")
        # check we are still on the ground...
        relative_alt = self.get_altitude(relative=True)
        if abs(relative_alt) > 0.1:
            raise NotAchievedException("Took off prematurely")
        self.progress("Pushing throttle past half-way")
        self.set_rc(3, 1650)

        self.progress("Monitoring takeoff")
        self.wait_altitude(6.9, 8, relative=True)

        self.progress("takeoff OK")

        self.land_and_disarm()
