'''
Plane autotest: TerrainLoiter

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TerrainLoiter(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def TerrainLoiter(self):
        '''Test terrain following in loiter'''
        self.context_push()
        self.install_terrain_handlers_context()
        self.set_parameters({
            "TERRAIN_FOLLOW": 1, # enable terrain following in loiter
            "WP_LOITER_RAD": 2000, # set very large loiter rad to get some terrain changes
        })
        alt = 200
        self.takeoff(alt*0.9, alt*1.1)
        self.set_rc(3, 1500)
        self.change_mode("LOITER")
        self.progress("loitering at %um" % alt)
        tstart = self.get_sim_time()
        timeout = 60*15  # enough time to do one and a bit circles
        max_delta = 0
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                break
            gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
            terrain = self.assert_receive_message('TERRAIN_REPORT')
            rel_alt = terrain.current_height
            self.progress("%um above terrain (%um bove home)" %
                          (rel_alt, gpi.relative_alt/1000.0))
            if rel_alt > alt*1.2 or rel_alt < alt * 0.8:
                raise NotAchievedException("Not terrain following")
            delta = abs(rel_alt - gpi.relative_alt/1000.0)
            if delta > max_delta:
                max_delta = delta
        want_max_delta = 30
        if max_delta < want_max_delta:
            raise NotAchievedException(
                "Expected terrain and home alts to vary more than they did (max=%u want=%u)" %
                (max_delta, want_max_delta))
        self.context_pop()
        self.progress("Returning home")
        self.fly_home_land_and_disarm(240)
