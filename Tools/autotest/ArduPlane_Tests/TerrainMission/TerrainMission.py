'''
Plane autotest: TerrainMission

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TerrainMission(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def TerrainMission(self):
        '''Test terrain following in mission'''
        self.install_terrain_handlers_context()

        num_wp = self.load_mission("ap-terrain.txt")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        global max_alt
        max_alt = 0

        def record_maxalt(mav, m):
            global max_alt
            if m.get_type()  != 'GLOBAL_POSITION_INT':
                return
            if m.relative_alt/1000.0 > max_alt:
                max_alt = m.relative_alt/1000.0

        self.install_message_hook_context(record_maxalt)

        self.fly_mission_waypoints(num_wp-1, mission_timeout=600)

        if max_alt < 200:
            raise NotAchievedException("Did not follow terrain")
