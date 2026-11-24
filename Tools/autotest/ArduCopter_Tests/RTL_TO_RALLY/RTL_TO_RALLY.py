'''
Copter autotest: RTL_TO_RALLY

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class RTL_TO_RALLY(TestGroup):
    @testcase(bucket="CopterTests2b")
    def RTL_TO_RALLY(self, target_system=1, target_component=1):
        '''Check RTL to rally point'''
        self.wait_ready_to_arm()
        rally_alt = 37
        rally_loc = self.offset_location_ne(self.home_position_as_location(), 50, -25)
        rally_loc.set_alt_m(rally_alt, AltFrame.ABOVE_HOME)
        items = [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(rally_loc.lat * 1e7), # latitude
                int(rally_loc.lng * 1e7), # longitude
                rally_alt, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
        ]
        self.upload_using_mission_protocol(
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
            items
        )
        self.set_parameters({
            'RALLY_INCL_HOME': 0,
        })
        self.takeoff(10, mode='GUIDED')
        self.hover()
        self.change_mode('RTL')
        self.wait_location(rally_loc, height_accuracy=None)
        self.assert_altitude(rally_alt, relative=True)
        self.progress("Ensuring we're descending")
        self.wait_altitude(20, 25, relative=True)
        self.change_mode('LOITER')
        self.progress("Flying home")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.change_mode('RTL')
        self.wait_disarmed()
        self.assert_at_home()
