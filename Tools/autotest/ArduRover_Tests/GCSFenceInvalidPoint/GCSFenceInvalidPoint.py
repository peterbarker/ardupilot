'''
Rover autotest: GCSFenceInvalidPoint

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GCSFenceInvalidPoint(TestGroup):
    @testcase(bucket="Rover")
    def GCSFenceInvalidPoint(self):
        '''Test fetch non-existent fencepoint'''
        target_system = 1
        target_component = 1

        here = self.get_location()
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
            ]),
        ])
        for i in 0, 1, 2:
            self.assert_fetch_mission_item_int(target_system, target_component, i, mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.progress("Requesting invalid point")
        self.mav.mav.mission_request_int_send(
            target_system,
            target_component,
            3,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )
        self.context_push()
        self.context_collect('MISSION_COUNT')
        m = self.assert_receive_mission_ack(
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE,
        )
        self.delay_sim_time(0.1, reason="mission count collection")
        found = False
        for m in self.context_collection('MISSION_COUNT'):
            if m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
                continue
            if m.count != 3:
                raise NotAchievedException("Unexpected count in MISSION_COUNT")
            found = True
        self.context_pop()
        if not found:
            raise NotAchievedException("Did not see correction for fencepoint count")
