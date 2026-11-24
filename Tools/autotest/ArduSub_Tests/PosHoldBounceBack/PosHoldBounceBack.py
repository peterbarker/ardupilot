'''
Sub autotest: PosHoldBounceBack

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase

from ardusub import Joystick


class PosHoldBounceBack(TestGroup):
    def wait_for_stop(self):
        """Watch the sub slow down and stop"""
        tstart = self.get_sim_time_cached()
        lstart = self.get_location()

        dmax = 0
        dprev = 0

        while True:
            self.delay_sim_time(1, reason="movement measurement interval")

            dcurr = self.get_distance(lstart, self.get_location())

            if dcurr - dmax < -0.2:
                raise NotAchievedException(f"Bounced back from {dmax:.2f}m to {dcurr:.2f}m")
            if dcurr > dmax:
                dmax = dcurr

            if abs(dcurr - dprev) < 0.1:
                self.progress(f"Stopping distance {dcurr:.2f}m, less than {self.get_sim_time_cached() - tstart:.2f}s")
                return

            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException("Took to long to stop")

            dprev = dcurr

    @testcase(bucket="Sub")
    def PosHoldBounceBack(self):
        """Test for bounce back in POSHOLD mode"""
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # dive a little
        self.set_rc(Joystick.Throttle, 1300)
        self.delay_sim_time(3, reason="vehicle to dive")
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(2, reason="vehicle to settle")

        # hold position
        self.change_mode('POSHOLD')

        for pilot_speed in range(50, 251, 100):
            # set max speed
            self.set_parameter('PILOT_SPEED', pilot_speed)

            # try different stick values, resulting speed is ~ max_speed * effort * gain
            for pwm in range(1700, 1901, 100):
                self.progress(f'PILOT_SPEED {pilot_speed}, forward pwm {pwm}')
                self.set_rc(Joystick.Forward, pwm)
                self.delay_sim_time(3, reason="forward movement")
                self.set_rc(Joystick.Forward, 1500)
                self.wait_for_stop()

        self.disarm_vehicle()
