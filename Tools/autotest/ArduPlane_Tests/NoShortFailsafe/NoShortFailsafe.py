'''
Plane autotest: NoShortFailsafe

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class NoShortFailsafe(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def NoShortFailsafe(self):
        '''No short failsafe in auto mode'''
        self.set_parameters({
            "FS_GCS_ENABL": 1, # GCS failsafe : heartbeat
            "FS_SHORT_ACTN": 0, # Short failsafe action: ignore if in auto|guided|loiter, otherwise circle
            "FS_LONG_ACTN": 0, # Long failsafe action: continue
            "RTL_AUTOLAND": 1,
            "MAV_GCS_SYSID": self.mav.source_system,
        })
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, 0),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 800, 0),
        ])
        self.wait_current_waypoint(2)
        self.progress("Disconnecting GCS")
        self.set_heartbeat_rate(0)
        self.delay_sim_time(10, reason="short failsafe window to pass")
        self.wait_mode("AUTO", timeout=10)
        self.set_heartbeat_rate(self.speedup)
        self.fly_home_land_and_disarm()
