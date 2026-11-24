'''
Rover autotest: MissionRetransfer

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MissionRetransfer(TestGroup):
    @testcase(bucket="Rover")
    def MissionRetransfer(self, target_system=1, target_component=1):
        '''torture-test with MISSION_COUNT'''
#        self.send_debug_trap()
        self.mav.mav.mission_count_send(
            target_system,
            target_component,
            10,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )
        self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_FENCE, 0)
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.mav.mav.mission_count_send(
            target_system,
            target_component,
            10000,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )
        self.wait_statustext('Only [0-9]+ items are supported', regex=True, check_context=True)
        self.context_pop()
        self.assert_not_receive_message('MISSION_REQUEST')
        self.mav.mav.mission_count_send(
            target_system,
            target_component,
            10,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        )
        self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_FENCE, 0)
        self.assert_receive_mission_ack(
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            want_type=mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED,
        )
