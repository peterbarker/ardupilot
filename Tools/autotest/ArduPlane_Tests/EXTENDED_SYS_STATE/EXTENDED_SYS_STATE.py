'''
Plane autotest: EXTENDED_SYS_STATE

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class EXTENDED_SYS_STATE(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def EXTENDED_SYS_STATE(self):
        '''check EXTENDED_SYS_STATE through a fixed-wing flight'''
        self.context_set_message_rate_hz('EXTENDED_SYS_STATE', 10)
        vtol_state = mavutil.mavlink.MAV_VTOL_STATE_UNDEFINED
        self.wait_extended_sys_state(vtol_state, mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND)

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 600, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_LAND, 10, 0, 0),
        ])
        for landed_state, timeout in [
                (mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF, 30),
                (mavutil.mavlink.MAV_LANDED_STATE_IN_AIR, 60),
                (mavutil.mavlink.MAV_LANDED_STATE_LANDING, 120),
                (mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 180),
        ]:
            self.wait_extended_sys_state(vtol_state, landed_state, timeout=timeout)
        self.wait_disarmed(timeout=60)
