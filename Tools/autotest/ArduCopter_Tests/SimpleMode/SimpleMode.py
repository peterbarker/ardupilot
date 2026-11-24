'''
Copter autotest: SimpleMode

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SimpleMode(TestGroup):
    #   SimpleMode - test simple mode flies North regardless of vehicle heading
    @testcase(bucket="CopterTests2b")
    def SimpleMode(self):
        '''Fly in SIMPLE mode'''

        # reboot to ensure previous test doesn't affect the initial heading
        self.reboot_sitl()

        # set SIMPLE mode for FlightMode2 (AltHold)
        self.set_parameters({
            "FLTMODE_CH": 5,
            "FLTMODE1": 5, # Loiter
            "FLTMODE2": 2, # AltHold
            "SIMPLE": 2,   # FLTMODE2 uses simple mode
        })

        # Takeoff in loiter
        self.takeoff(10, mode="LOITER")

        # Fail immediately if heading is not the expected 270
        self.wait_heading(270, 5, timeout=60)

        # Try a range of headings
        for yaw_angle in [0, 90, 180, 270]:
            self.progress("Testing SIMPLE mode with copter yaw=%u degrees" % yaw_angle)

            # yaw to test angle
            self.set_rc(4, 1560)
            self.wait_heading(yaw_angle, timeout=60)
            self.set_rc(4, 1500)

            # switch to alt hold mode
            self.set_rc(5, 1298)

            # RC input to roll right towards North
            start = self.get_location()
            self.set_rc(1, 1700)
            self.wait_distance(50)
            self.set_rc(1, 1500)

            # verify ground course is approximately north
            end = self.get_location()
            bearing = self.get_bearing(start, end)
            if self.heading_delta(bearing, 0) > 10:
                raise NotAchievedException(
                    "SIMPLE mode yaw=%u: ground course %f, want ~0 (north)" %
                    (yaw_angle, bearing)
                )

            # Loiter to a stop before next iteration
            self.set_rc(5, 1165)
            self.wait_groundspeed(0, 0.1)

        self.do_RTL(timeout=500)
