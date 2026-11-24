'''
Rover autotest: SDPolyFence

AP_FLAKE8_CLEAN
'''

from pymavlink import mavextra
from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class SDPolyFence(TestGroup):
    # this method modified from cmd_addpoly in the MAVProxy code:
    def generate_polyfence(self, centre_loc, command, radius, count, rotation=0):
        '''adds a number of waypoints equally spaced around a circle

        '''
        if count < 3:
            raise ValueError("Invalid count (%s)" % str(count))
        if radius <= 0:
            raise ValueError("Invalid radius (%s)" % str(radius))

        latlon = (centre_loc.lat, centre_loc.lng)

        items = []
        for i in range(0, count):
            (lat, lon) = mavextra.gps_newpos(latlon[0],
                                             latlon[1],
                                             360/float(count)*i + rotation,
                                             radius)

            m = mavutil.mavlink.MAVLink_mission_item_int_message(
                1,  # target system
                1,  # target component
                0,    # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL,    # frame
                command,    # command
                0,    # current
                0,    # autocontinue
                count, # param1,
                0.0,  # param2,
                0.0,  # param3
                0.0,  # param4
                int(lat*1e7),  # x (latitude)
                int(lon*1e7),  # y (longitude)
                0,                     # z (altitude)
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            )
            items.append(m)

        return items

    @testcase(bucket="Rover")
    def SDPolyFence(self):
        '''test storage of fence on SD card'''
        self.set_parameters({
            'BRD_SD_FENCE': 32767,
        })
        self.reboot_sitl()

        home = self.home_position_as_location()
        fence = self.generate_polyfence(
            home,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            radius=100,
            count=100,
        )

        for bearing in range(0, 359, 60):
            x = self.offset_location_heading_distance(home, bearing, 100)
            fence.extend(self.generate_polyfence(
                x,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                radius=100,
                count=100,
            ))

        self.correct_wp_seq_numbers(fence)
        self.check_fence_upload_download(fence)
