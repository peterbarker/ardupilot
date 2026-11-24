'''
Copter autotest: ModeCircle

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ModeCircle(TestGroup):
    # fly_circle - flies a circle with 20m radius
    @testcase(bucket="CopterTests2a")
    def ModeCircle(self, holdtime=36):
        '''Fly CIRCLE mode'''
        # the following should not be required.  But there appears to
        # be a physics failure in the simulation which is causing CI
        # to fall over a lot.  -pb 202007021209
        self.reboot_sitl()

        self.takeoff(10, mode="LOITER")

        # face west
        self.progress("turn west")
        self.set_rc(4, 1580)
        self.wait_heading(270)
        self.set_rc(4, 1500)

        # set CIRCLE radius
        self.set_parameter("CIRCLE_RADIUS_M", 30)

        # fly forward (east) at least 100m
        self.set_rc(2, 1100)
        self.wait_distance(100)
        # return pitch stick back to middle
        self.set_rc(2, 1500)

        # set CIRCLE mode
        self.change_mode('CIRCLE')

        # wait
        m = self.assert_receive_message('VFR_HUD')
        start_altitude = m.alt
        tstart = self.get_sim_time()
        self.progress("Circle at %u meters for %u seconds" %
                      (start_altitude, holdtime))
        while self.get_sim_time_cached() < tstart + holdtime:
            m = self.assert_receive_message('VFR_HUD')
            self.progress("heading %d" % m.heading)

        self.progress("CIRCLE OK for %u seconds" % holdtime)

        self.do_RTL()
