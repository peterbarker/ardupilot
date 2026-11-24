'''
Plane autotest: FenceDoubleBreach

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class FenceDoubleBreach(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def FenceDoubleBreach(self):
        '''test breaching the fence twice'''
        self.wait_ready_to_arm()

        fence_centre_ne = (0, -500)

        fence_centre = self.get_location()
        fence_centre = self.offset_location_ne(fence_centre, fence_centre_ne[0], fence_centre_ne[1])

        self.set_parameters({
            "RTL_AUTOLAND": 2,
        })

        alt = 50
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 200, 0, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, fence_centre_ne[0], fence_centre_ne[1], alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -750, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -1000, alt),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_LAND_START,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 800, alt),
        ])

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                self.offset_location_ne(fence_centre, -200, -200), # bl
                self.offset_location_ne(fence_centre, -200, 200), # br
                self.offset_location_ne(fence_centre, 200, 200), # tr
                self.offset_location_ne(fence_centre, 200, -200), # tl,
            ]),
        ])

        self.do_fence_enable()

        self.takeoff(mode='FBWA')
        self.set_rc(3, 1500)

        self.change_mode('AUTO')

        self.context_collect('STATUSTEXT')

        self.wait_statustext('Polygon fence breached', timeout=300)
        self.wait_current_waypoint(6)
        self.wait_distance_to_location(fence_centre, 350, 20000)

        self.set_current_waypoint(2)

        self.wait_statustext('Polygon fence breached', timeout=300)
        self.wait_current_waypoint(6, timeout=5)
        self.fly_home_land_and_disarm()
