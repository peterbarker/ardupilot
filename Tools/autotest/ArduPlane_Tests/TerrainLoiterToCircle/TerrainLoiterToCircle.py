'''
Plane autotest: TerrainLoiterToCircle

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class TerrainLoiterToCircle(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def TerrainLoiterToCircle(self):
        '''loiter terrain-relative.  Switch to Circle, maintain alt'''
        self.install_terrain_handlers_context()
        self.set_parameters({
            "TERRAIN_FOLLOW": 1, # enable terrain following in loiter
            "WP_LOITER_RAD": 2000, # set very large loiter rad to get some terrain changes
        })
        alt = 50
        self.takeoff(alt*0.9, alt*1.1, mode='TAKEOFF')
        self.change_mode('LOITER')
        self.delay_sim_time(10, reason="loiter to establish terrain following")
        self.change_mode('CIRCLE')
        self.wait_altitude(alt*0.9, alt*1.1, minimum_duration=10,
                           altitude_source="TERRAIN_REPORT.current_height")
        self.fly_home_land_and_disarm()
