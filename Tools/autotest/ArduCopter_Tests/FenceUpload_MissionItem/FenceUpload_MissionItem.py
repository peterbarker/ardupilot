'''
Copter autotest: FenceUpload_MissionItem

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FenceUpload_MissionItem(TestGroup):
    @testcase(bucket="CopterTests2b")
    def FenceUpload_MissionItem(self, timeout=180):
        '''Test MISSION_ITEM fence upload/download'''
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 6,  # polygon and circle fences
        })

        self.poll_home_position(quiet=False)

        home_loc = self.get_location()

        fence_loc = [
            self.offset_location_ne(home_loc, -110, -110),
            self.offset_location_ne(home_loc, 110, -110),
            self.offset_location_ne(home_loc, 110, 110),
            self.offset_location_ne(home_loc, -110, 110),
        ]

        seq = 0
        items = []
        mission_type = mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        count = len(fence_loc)
        for loc in fence_loc:
            item = self.mav.mav.mission_item_encode(
                1,
                1,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, 0,
                count, 0, 0, 0,
                loc.lat, loc.lng, 33.0,
                mission_type
            )
            items.append(item)
            seq += 1

        self.upload_using_mission_protocol(mission_type, items)
        downloaded_items = self.download_using_mission_protocol(mission_type)

        if len(downloaded_items) != len(items):
            raise NotAchievedException(f"Mismatch in number of items: sent={len(items)} received={len(downloaded_items)}")

        for i, (sent, received) in enumerate(zip(items, downloaded_items)):
            mismatches = []

            # Normalize lat/lon to float before comparison
            sent_lat = sent.x
            sent_lng = sent.y
            recv_lat = received.x / 1e7 if isinstance(received.x, int) else received.x
            recv_lng = received.y / 1e7 if isinstance(received.y, int) else received.y

            if sent.command != received.command:
                mismatches.append(f"command: {sent.command} != {received.command}")
            if not math.isclose(sent_lat, recv_lat, abs_tol=1e-2):
                mismatches.append(f"lat: {sent_lat} != {recv_lat}")
            if not math.isclose(sent_lng, recv_lng, abs_tol=1e-2):
                mismatches.append(f"lng: {sent_lng} != {recv_lng}")
            if not math.isclose(sent.param1, received.param1, abs_tol=1e-3):
                mismatches.append(f"param1: {sent.param1} != {received.param1}")

            if mismatches:
                raise NotAchievedException(f"Mismatch in item {i}: " + "; ".join(mismatches))

        print("Fence upload/download verification passed.")
