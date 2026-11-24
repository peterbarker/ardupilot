'''
Rover autotest: PositionTargetGlobalIntAltFrame

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase

from rover import SITL_START_LOCATION


class PositionTargetGlobalIntAltFrame(TestGroup):
    @testcase(bucket="Rover")
    def PositionTargetGlobalIntAltFrame(self):
        '''Test that POSITION_TARGET_GLOBAL_INT sends AMSL altitude'''
        # Use MAV_CMD_NAV_LOITER_TURNS to trigger SubMode::Circle in AUTO
        # mode with a 0m altitude in MAV_FRAME_GLOBAL_RELATIVE_ALT_INT.
        # At the SITL start location (~1584m AMSL) the correct AMSL altitude
        # is ~1584m, not 0m.
        home_alt_amsl = SITL_START_LOCATION.get_alt_m(AltFrame.ABSOLUTE)  # ~1583.7m

        home_loc = self.home_position_as_location()
        # NAV_LOITER_TURNS: param1=number of turns, param3=radius in metres
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 10, 0, 0),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                p1=100,  # number of turns (large to keep rover circling)
                p3=500,  # radius in metres
                x=int(home_loc.lat * 1e7),
                y=int(home_loc.lng * 1e7),
                z=0,     # 0m relative to home
            ),
        ])
        self.set_current_waypoint(0, check_afterwards=False)

        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Wait for the rover to reach and begin the circle waypoint
        self.wait_current_waypoint(2, timeout=60)

        # Wait for POSITION_TARGET_GLOBAL_INT to report the correct AMSL altitude
        self.wait_message_field_values("POSITION_TARGET_GLOBAL_INT", {
            "alt": home_alt_amsl,
        }, epsilon=10, timeout=30)

        self.disarm_vehicle()
