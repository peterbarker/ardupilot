'''
Rover autotest: FenceFullAndPartialTransfer

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class FenceFullAndPartialTransfer(TestGroup):
    @testcase(bucket="Rover")
    def FenceFullAndPartialTransfer(self, target_system=1, target_component=1):
        '''ensure starting a fence transfer then a partial transfer behaves
        appropriately'''
        # start uploading a 10 item list:
        self.mav.mav.mission_count_send(
            target_system,
            target_component,
            10,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )
        self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_FENCE, 0)
        # change our mind and try a partial mission upload:
        self.mav.mav.mission_write_partial_list_send(
            target_system,
            target_component,
            3,
            3,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        # should get denied for that one:
        self.assert_receive_mission_ack(
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            want_type=mavutil.mavlink.MAV_MISSION_DENIED,
        )
        # now wait for the original upload to be "cancelled"
        self.assert_receive_mission_ack(
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            want_type=mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED,
        )
