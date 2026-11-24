'''
Plane autotest: LOITER

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class LOITER(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def LOITER(self):
        '''Test Loiter mode'''
        # first test old loiter behaviour
        self.set_parameter("FLIGHT_OPTIONS", 0)
        self.takeoff(alt=200)
        self.set_rc(3, 1500)
        self.change_mode("LOITER")
        self.progress("Doing a bit of loitering to start with")
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 60:
                break
            m = self.assert_receive_message('VFR_HUD')
            new_throttle = m.throttle
            alt = m.alt
            m = self.assert_receive_message('ATTITUDE', timeout=5)
            pitch = math.degrees(m.pitch)
            self.progress("Pitch:%f throttle:%u alt:%f" % (pitch, new_throttle, alt))
        m = self.assert_receive_message('VFR_HUD', timeout=5)
        initial_throttle = m.throttle
        initial_alt = m.alt
        self.progress("Initial throttle: %u" % initial_throttle)
        # pitch down, ensure throttle decreases:
        rc2_max = self.get_parameter("RC2_MAX")
        self.set_rc(2, int(rc2_max))
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            '''stick-mixing is pushing the aircraft down.  It doesn't want to go
            down (the target loiter altitude hasn't changed), so it
            tries to add energy by increasing the throttle.
            '''
            if now - tstart > 60:
                raise NotAchievedException("Did not see increase in throttle")
            m = self.assert_receive_message('VFR_HUD', timeout=5)
            new_throttle = m.throttle
            alt = m.alt
            m = self.assert_receive_message('ATTITUDE', timeout=5)
            pitch = math.degrees(m.pitch)
            self.progress("Pitch:%f throttle:%u alt:%f" % (pitch, new_throttle, alt))
            if new_throttle - initial_throttle > 20:
                self.progress("Throttle delta achieved")
                break
        self.progress("Centering elevator and ensuring we get back to loiter altitude")
        self.set_rc(2, 1500)
        self.wait_altitude(initial_alt-1, initial_alt+1)
        # Test new loiter behaviour
        self.set_parameter("FLIGHT_OPTIONS", 1 << 12)
        # should descend at max stick
        self.set_rc(2, int(rc2_max))
        self.wait_altitude(initial_alt - 110, initial_alt - 90, timeout=90)
        # should not climb back at mid stick
        self.set_rc(2, 1500)
        self.delay_sim_time(60, reason="altitude hold verification")
        self.wait_altitude(initial_alt - 110, initial_alt - 90)
        # should climb at min stick
        self.set_rc(2, 1100)
        self.wait_altitude(initial_alt - 10, initial_alt + 10, timeout=90)
        # return stick to center and fly home
        self.set_rc(2, 1500)
        self.fly_home_land_and_disarm()
