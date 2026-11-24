'''
Rover autotest: MAV_CMD_DO_FENCE_ENABLE

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_FENCE_ENABLE(TestGroup):
    @testcase(bucket="Rover")
    def MAV_CMD_DO_FENCE_ENABLE(self):
        '''ensure MAV_CMD_DO_FENCE_ENABLE mavlink command works'''
        here = self.get_location()

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, -50, -50), # bl
                self.offset_location_ne(here, -50, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, -50), # tl,
            ]),
        ])

        # enable:
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, p1=1)
        self.assert_fence_enabled()

        # disable
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, p1=0)
        self.assert_fence_disabled()
