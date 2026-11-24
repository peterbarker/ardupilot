'''
QuadPlane autotest: RudderArmedTakeoffRequiresNeutralThrottle

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class RudderArmedTakeoffRequiresNeutralThrottle(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def RudderArmedTakeoffRequiresNeutralThrottle(self):
        '''check rudder must be neutral before VTOL takeoff allowed'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, 0, 0, 10),
            (mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND, 0, 0, 0),
        ])
        self.upload_simple_relhome_mission
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.set_rc(4, 2000)
        self.wait_armed()
        self.wait_altitude(-1, 1, relative=True, minimum_duration=10)
        self.set_rc(4, 1500)
        self.wait_altitude(5, 1000, relative=True)
        self.zero_throttle()
        self.wait_disarmed(timeout=60)
