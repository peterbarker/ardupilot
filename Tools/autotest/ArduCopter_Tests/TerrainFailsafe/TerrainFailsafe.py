'''
Copter autotest: TerrainFailsafe

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class TerrainFailsafe(TestGroup):
    @testcase(bucket="CopterTests2c")
    def TerrainFailsafe(self):
        '''test that auto mode triggers terrain failsafe if waypoint alt frame is terrain and terrain database is disabled'''
        # allow arming and takeoff in Auto mode
        self.set_parameter('AUTO_OPTIONS', 3)

        self.install_terrain_handlers_context()

        # create a mission: takeoff to 20m then waypoint with alt-above-terrain
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 100, 0, 0, {
                "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
            }),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        # change to auto and wait for arming checks to pass
        self.change_mode('AUTO')
        self.wait_ready_to_arm(timeout=200)

        # disable terrain database and force arm vehicle
        self.set_parameter('TERRAIN_ENABLE', 0)
        self.arm_vehicle(force=True)

        # takeoff should succeed but WP with terrain alt should trigger
        # the terrain failsafe and vehicle should switch to RTL
        self.wait_statustext("Failsafe: Terrain", timeout=60)
        self.wait_mode("RTL")
        self.wait_rtl_complete()
