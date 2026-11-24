'''
Copter autotest: SuperSimpleCircle

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SuperSimpleCircle(TestGroup):
    # fly_super_simple - flies a circle around home for 45 seconds
    @testcase(bucket="CopterTests1d")
    def SuperSimpleCircle(self, timeout=45):
        '''Fly a circle in SUPER SIMPLE mode'''
        self.takeoff(10, mode="LOITER")

        # fly forward 20m
        self.progress("# Flying forward 20 meters")
        self.set_rc(2, 1300)
        self.wait_distance(20, 5, 60)
        self.set_rc(2, 1500)

        # set SUPER SIMPLE mode for all flight modes
        self.set_parameter("SUPER_SIMPLE", 63)

        # switch to stabilize mode
        self.change_mode("ALT_HOLD")
        self.set_rc(3, 1500)

        # start copter yawing slowly
        self.set_rc(4, 1550)

        # roll left for timeout seconds
        self.progress("# rolling left from pilot's POV for %u seconds"
                      % timeout)
        self.set_rc(1, 1300)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < (tstart + timeout):
            self.assert_receive_message('VFR_HUD')

        # stop rolling and yawing
        self.set_rc_from_map({1: 1500, 4: 1500})

        # restore simple mode parameters to default
        self.set_parameter("SUPER_SIMPLE", 0)

        # hover in place
        self.hover()

        self.do_RTL()
