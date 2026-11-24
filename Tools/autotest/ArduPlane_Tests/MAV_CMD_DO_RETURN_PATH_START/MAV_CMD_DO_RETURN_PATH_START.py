'''
Plane autotest: MAV_CMD_DO_RETURN_PATH_START

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_RETURN_PATH_START(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def MAV_CMD_DO_RETURN_PATH_START(self):
        '''test MAV_CMD_DO_RETURN_PATH_START as a mavlink command'''
        self.start_subtest("refused without a return path in the mission")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, 50),
        ])
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START, want_result=mavutil.mavlink.MAV_RESULT_FAILED)

        # a mission with DO_RETURN_PATH_START fails prearms with RTL_AUTOLAND disabled:
        self.set_parameter("RTL_AUTOLAND", 4)

        self.start_subtest("does not switch away from a return path being flown")
        # the leg from item 5 to item 6 passes 100m from item 2:
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),     # 1
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 100, 400, 50),  # 2
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START),  # 3
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 800, 50),  # 4
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 800, 50),    # 5
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 50),      # 6
        ])
        self.wait_current_waypoint(4, timeout=120)
        self.context_collect('STATUSTEXT')
        self.context_collect('MISSION_CURRENT')
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START)
        self.wait_statustext("Return path active", check_context=True)
        self.delay_sim_time(2, reason="any rejoin to take effect")
        # not rejoined at the nearby leg to item 6:
        for m in self.context_collection('MISSION_CURRENT'):
            if m.seq == 6:
                raise NotAchievedException("Switched away from the return path being flown")

        self.start_subtest("refused if AUTO may not be entered from the GCS")
        self.change_mode('GUIDED')
        # beside the leg from item 4 to item 5:
        loc = self.offset_location_ne(self.home_position_as_location(), 400, 900)
        loc.set_alt_m(50, AltFrame.ABOVE_HOME)
        self.send_do_reposition(loc)
        self.wait_location(loc, accuracy=200, height_accuracy=None, timeout=120)
        self.set_parameter("FLTMODE_GCSBLOCK", 1 << 9)  # AUTO
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.assert_mode_is('GUIDED')
        self.set_parameter("FLTMODE_GCSBLOCK", 0)

        self.start_subtest("joins the closest leg of the return path")
        # the refused request has already moved the current item to the
        # joining point; move it away so this request has to do it:
        self.set_current_waypoint(2)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START)
        self.wait_mode('AUTO')
        self.wait_current_waypoint(5, timeout=10)

        self.fly_home_land_and_disarm()
