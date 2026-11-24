'''
Sub autotest: TerrainMission

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class TerrainMission(TestGroup):
    @testcase(bucket="Sub")
    def TerrainMission(self):
        """Mission using surface tracking"""

        self.assert_parameter_value('RNGFND1_MAX', 30)

        filename = "terrain_mission.txt"
        self.progress(f"Executing mission {filename}")
        self.load_mission(filename)
        self.set_rc_default()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_waypoint(1, 4, max_dist_to_final_wp_m=5)

        # Expect sub to hover at final altitude
        self.wait_altitude(altitude_min=-37, altitude_max=-35, relative=False, minimum_duration=2, timeout=30)

        self.disarm_vehicle()
        self.progress("Mission OK")
