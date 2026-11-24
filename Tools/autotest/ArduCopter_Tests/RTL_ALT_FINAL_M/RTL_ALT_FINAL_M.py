'''
Copter autotest: RTL_ALT_FINAL_M

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RTL_ALT_FINAL_M(TestGroup):
    @testcase(bucket="CopterTests2b")
    def RTL_ALT_FINAL_M(self):
        '''Test RTL with RTL_ALT_FINAL_M'''
        self.progress("arm the vehicle and takeoff in Guided")
        self.takeoff(20, mode='GUIDED')

        # Set home current location, this gives a large home vs origin difference
        self.set_home(self.get_location())

        self.progress("fly 50m North (or whatever)")
        self.fly_guided_move_local(50, 0, 50)
        target_alt = 10
        self.set_parameter('RTL_ALT_FINAL_M', target_alt)

        self.progress("Waiting RTL to reach Home and hold")
        self.change_mode('RTL')

        # Expecting to return and hold 10m above home
        tstart = self.get_sim_time()
        reachedHome = False
        while self.get_sim_time_cached() < tstart + 250:
            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            alt = m.relative_alt / 1000.0 # mm -> m
            home_distance = self.distance_to_home(use_cached_home=True)
            home = math.sqrt((alt-target_alt)**2 + home_distance**2) < 2
            if home and not reachedHome:
                reachedHome = True
                self.progress("Reached home - holding")
                self.delay_sim_time(20, reason="RTL hold at home")
                continue

            if reachedHome:
                if not home:
                    raise NotAchievedException("Should still be at home")
                if not self.armed():
                    raise NotAchievedException("Should still be armed")
                break

        self.progress("Hold at home successful - landing")
        self.change_mode("LAND")
        self.wait_landed_and_disarmed()
