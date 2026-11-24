'''
Sub autotest: AutoTerrainRecover

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class AutoTerrainRecover(TestGroup):
    @testcase(bucket="Sub")
    def AutoTerrainRecover(self):
        """Test Auto_TerrainRecover"""
        self.set_parameter('RNGFND1_MAX', 50)

        # Start at (0, 0, -5)
        self.dive(-5, mode='ALT_HOLD')

        # Move to (50, 0, 20) in above-terrain frame
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 50, 0, 20, {
                "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
            }),
        ])

        self.change_mode('AUTO')
        self.delay_sim_time(10, reason="Wait for mission to start")

        # Reduce rangefinder range to trigger failsafe recovery
        self.context_collect('STATUSTEXT')
        self.set_parameter("RNGFND1_MAX", 35)
        self.wait_statustext("Attempting auto failsafe recovery")

        # The vehicle will dive until it recovers
        self.wait_statustext("Terrain failsafe recovery successful!")

        # Reduce it again, this time recovery will time out, and the vehicle will automatically disarm
        self.set_parameter("RNGFND1_MAX", 15)
        self.wait_statustext("Terrain failsafe recovery timeout!")
        self.wait_statustext("Disarming motors")
