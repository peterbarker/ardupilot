'''
Copter autotest: SplineTerrain

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SplineTerrain(TestGroup):
    @testcase(bucket="CopterTests1c")
    def SplineTerrain(self):
        '''Test Splines and Terrain'''
        self.set_parameter("TERRAIN_ENABLE", 0)
        self.fly_mission("wp.txt")
