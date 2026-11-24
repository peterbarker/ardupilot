'''
Rover autotest: ClearMission

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class ClearMission(TestGroup):
    @testcase(bucket="Rover")
    def ClearMission(self, target_system=1, target_component=1):
        '''check mission clearing'''

        self.start_subtest("Clear via mission_clear_all message")
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
        ])
        self.set_current_waypoint(3)

        self.mav.mav.mission_clear_all_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

        self.assert_current_waypoint(0)

        self.drain_mav()

        self.start_subtest("No clear mission while it is being uploaded by a different node")
        mav2 = mavutil.mavlink_connection(self.sitl_serial_endpoint(2),
                                          robust_parsing=True,
                                          source_system=7,
                                          source_component=7)
        self.context_push()
        self.context_collect("MISSION_REQUEST")
        mav2.mav.mission_count_send(target_system,
                                    target_component,
                                    17,
                                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        ack = self.assert_receive_message('MISSION_REQUEST', check_context=True, mav=mav2)
        self.context_pop()

        self.context_push()
        self.context_collect("MISSION_ACK")
        self.mav.mav.mission_clear_all_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        ack = self.assert_receive_message('MISSION_ACK', check_context=True)
        self.assert_message_field_values(ack, {
            "type": mavutil.mavlink.MAV_MISSION_DENIED,
        })
        self.context_pop()

        self.progress("Test cancel upload from second connection")
        self.context_push()
        self.context_collect("MISSION_ACK")
        mav2.mav.mission_clear_all_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        ack = self.assert_receive_message('MISSION_ACK', mav=mav2, check_context=True)
        self.assert_message_field_values(ack, {
            "type": mavutil.mavlink.MAV_MISSION_ACCEPTED,
        })
        self.context_pop()
        mav2.close()
        del mav2
