'''
Copter autotest: LoiterCoordinatedTurn

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class LoiterCoordinatedTurn(TestGroup):
    @testcase(bucket="CopterTests2b")
    def LoiterCoordinatedTurn(self):
        '''Loiter coordinated turns do not surge after releasing pitch and yaw'''
        self.set_parameters({
            "LOIT_OPTIONS": 1,
        })
        self.takeoff(20, mode='LOITER')

        self.set_rc_from_map({
            2: 1000,
            4: 2000,
        })
        self.delay_sim_time(10, reason="turn at full pitch and full yaw")
        release_loc = self.get_location()
        release_speed = self.assert_receive_message('VFR_HUD').groundspeed
        self.set_rc_from_map({
            2: 1500,
            4: 1500,
        })
        max_speed = release_speed
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 30:
                raise NotAchievedException("did not stop")
            speed = self.assert_receive_message('VFR_HUD').groundspeed
            max_speed = max(max_speed, speed)
            if speed < 0.2:
                break
        stop_dist = self.get_distance(release_loc, self.get_location())
        self.progress("release speed %.2f m/s, peak after release %.2f m/s, stopped %.1f m from release" %
                      (release_speed, max_speed, stop_dist))
        if max_speed > release_speed + 0.5:
            raise NotAchievedException("speed rose to %.2f m/s after release at %.2f m/s" % (max_speed, release_speed))
        if stop_dist > 4:
            raise NotAchievedException("stopped %.1f m from release, want under 4" % stop_dist)

        self.do_RTL()
