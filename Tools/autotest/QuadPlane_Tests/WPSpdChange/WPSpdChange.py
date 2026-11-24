'''
QuadPlane autotest: WPSpdChange

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class WPSpdChange(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def WPSpdChange(self):
        '''verify Q_WP_SPD takes effect on AUTO entry without reboot'''
        # enable VTOL-only AUTO so Q_WP_SPD controls cruise speed
        self.set_parameter('Q_ENABLE', 2)
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 700, 0, 40),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -700, 0, 40),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 700, 0, 40),
        ])

        # set Q_WP_SPD above default (5) without rebooting after the change
        self.set_parameter('Q_WP_SPD', 12.0)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')

        # wait for VTOL takeoff to complete, then check cruise speed
        self.wait_altitude(35, 45, relative=True, timeout=60)
        self.wait_groundspeed(8, 16, timeout=60)

        # switch to QRTL so the plane ends up where it started
        self.change_mode('QRTL')
        self.wait_disarmed(timeout=120)
