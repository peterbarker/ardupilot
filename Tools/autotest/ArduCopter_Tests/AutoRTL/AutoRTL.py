'''
Copter autotest: AutoRTL

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import WaitModeTimeout

from test_discovery import TestGroup
from test_discovery import testcase


class AutoRTL(TestGroup):
    @testcase(bucket="CopterTests2a")
    def AutoRTL(self):
        '''Test Auto RTL mode using do land start and return path start mission items'''
        alt = 50
        guided_loc = self.offset_location_ne(self.home_position_as_location(), 1000, 0)
        guided_loc.set_alt_m(alt, AltFrame.ABOVE_HOME)

        # Arm, take off and fly to guided location
        self.takeoff(mode='GUIDED')
        self.fly_guided_move_to(guided_loc, timeout=240)

        # Try auto RTL mode, should fail with no mission
        try:
            self.change_mode('AUTO_RTL', timeout=10)
            raise NotAchievedException("Should not change mode with no mission")
        except WaitModeTimeout:
            pass
        except Exception as e:
            raise e

        # pymavlink does not understand the new return path command yet, at some point it will
        cmd_return_path_start = 188 # mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START

        # Do land start and do return path should both fail as commands too
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_LAND_START, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.run_cmd(cmd_return_path_start, want_result=mavutil.mavlink.MAV_RESULT_FAILED)

        # Load mission with do land start
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1000, 0, alt), # 1
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  750, 0, alt), # 2
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_LAND_START), # 3
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, alt),  # 4
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 250, 0, alt),  # 5
        ])

        # Return path should still fail
        self.run_cmd(cmd_return_path_start, want_result=mavutil.mavlink.MAV_RESULT_FAILED)

        # Do land start should jump to the waypoint following the item
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_LAND_START, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)
        self.drain_mav()
        self.assert_current_waypoint(4)

        # Back to guided location
        self.change_mode('GUIDED')
        self.fly_guided_move_to(guided_loc)

        # mode change to Auto RTL should do the same
        self.change_mode('AUTO_RTL')
        self.drain_mav()
        self.assert_current_waypoint(4)

        # Back to guided location
        self.change_mode('GUIDED')
        self.fly_guided_move_to(guided_loc)

        # Add a return path item
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1250, 0, alt), # 1
            self.create_MISSION_ITEM_INT(cmd_return_path_start),  # 2
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 900, 0, alt),  # 3
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 750, 0, alt),  # 4
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 550, 0, alt),  # 5
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, alt),  # 6
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_LAND_START), # 7
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  250, 0, alt), # 8
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -250, 0, alt), # 9
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, -500, 0, alt), # 10
        ])

        # Return path should now work
        self.run_cmd(cmd_return_path_start, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)
        self.drain_mav()
        self.assert_current_waypoint(3)

        # Back to guided location
        self.change_mode('GUIDED')
        self.fly_guided_move_to(guided_loc)

        # mode change to Auto RTL should join the return path
        self.change_mode('AUTO_RTL')
        self.drain_mav()
        self.assert_current_waypoint(3)

        # do land start should still work
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_LAND_START, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)
        self.drain_mav()
        self.assert_current_waypoint(8)

        # Move a bit closer in guided
        return_path_test = self.offset_location_ne(self.home_position_as_location(), 600, 0)
        return_path_test.set_alt_m(alt, AltFrame.ABOVE_HOME)
        self.change_mode('GUIDED')
        self.fly_guided_move_to(return_path_test, timeout=100)

        # check the mission is joined further along
        self.run_cmd(cmd_return_path_start, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)
        self.drain_mav()
        self.assert_current_waypoint(5)

        # fly over home
        home = self.home_position_as_location()
        home.set_alt_m(alt, AltFrame.ABOVE_HOME)
        self.change_mode('GUIDED')
        self.fly_guided_move_to(home, timeout=140)

        # Should never join return path after do land start
        self.run_cmd(cmd_return_path_start, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)
        self.drain_mav()
        self.assert_current_waypoint(6)

        # Done
        self.land_and_disarm()
