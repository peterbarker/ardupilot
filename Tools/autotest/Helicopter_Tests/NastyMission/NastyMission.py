'''
Helicopter autotest: NastyMission

AP_FLAKE8_CLEAN
'''

import copy

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class NastyMission(TestGroup):
    def scurve_nasty_mission(self, target_system=1, target_component=1):
        '''returns a mission which attempts to give the SCurve library
        indigestion.  The same destination is given several times.'''

        wp2_loc = self.get_location()
        wp2_offset_n = 20
        wp2_offset_e = 30
        self.location_offset_ne(wp2_loc, wp2_offset_n, wp2_offset_e)

        wp2_by_three = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            2, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, # current
            0, # autocontinue
            3, # p1
            0, # p2
            0, # p3
            0, # p4
            int(wp2_loc.lat * 1e7), # latitude
            int(wp2_loc.lng * 1e7), # longitude
            31.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        wp5_loc = self.get_location()
        wp5_offset_n = -20
        wp5_offset_e = 30
        self.location_offset_ne(wp5_loc, wp5_offset_n, wp5_offset_e)

        wp5_by_three = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            5, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
            0, # current
            0, # autocontinue
            3, # p1
            0, # p2
            0, # p3
            0, # p4
            int(wp5_loc.lat * 1e7), # latitude
            int(wp5_loc.lng * 1e7), # longitude
            31.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        ret = copy.copy([
            # slot 0 is home
            self.mission_item_home(target_system=target_system, target_component=target_component),
            # slot 1 is takeoff
            self.mission_item_copter_takeoff(target_system=target_system, target_component=target_component),
            # now three spline waypoints right on top of one another:
            copy.copy(wp2_by_three),
            copy.copy(wp2_by_three),
            copy.copy(wp2_by_three),
            # now three MORE spline waypoints right on top of one another somewhere else:
            copy.copy(wp5_by_three),
            copy.copy(wp5_by_three),
            copy.copy(wp5_by_three),
            self.mission_item_rtl(target_system=target_system, target_component=target_component),
        ])
        self.correct_wp_seq_numbers(ret)
        return ret

    def scurve_nasty_up_mission(self, target_system=1, target_component=1):
        '''returns a mission which attempts to give the SCurve library
        indigestion.  The same destination is given several times but with differing altitudes.'''

        wp2_loc = self.get_location()
        wp2_offset_n = 20
        wp2_offset_e = 30
        self.location_offset_ne(wp2_loc, wp2_offset_n, wp2_offset_e)

        wp2 = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            2, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, # current
            0, # autocontinue
            3, # p1
            0, # p2
            0, # p3
            0, # p4
            int(wp2_loc.lat * 1e7), # latitude
            int(wp2_loc.lng * 1e7), # longitude
            31.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        wp3 = copy.copy(wp2)
        wp3.alt = 40
        wp4 = copy.copy(wp2)
        wp4.alt = 31

        wp5_loc = self.get_location()
        wp5_offset_n = -20
        wp5_offset_e = 30
        self.location_offset_ne(wp5_loc, wp5_offset_n, wp5_offset_e)

        wp5 = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            5, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
            0, # current
            0, # autocontinue
            3, # p1
            0, # p2
            0, # p3
            0, # p4
            int(wp5_loc.lat * 1e7), # latitude
            int(wp5_loc.lng * 1e7), # longitude
            31.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        wp6 = copy.copy(wp5)
        wp6.alt = 41
        wp7 = copy.copy(wp5)
        wp7.alt = 51

        ret = copy.copy([
            # slot 0 is home
            self.mission_item_home(target_system=target_system, target_component=target_component),
            # slot 1 is takeoff
            self.mission_item_copter_takeoff(target_system=target_system, target_component=target_component),
            wp2,
            wp3,
            wp4,
            # now three MORE spline waypoints right on top of one another somewhere else:
            wp5,
            wp6,
            wp7,
            self.mission_item_rtl(target_system=target_system, target_component=target_component),
        ])
        self.correct_wp_seq_numbers(ret)
        return ret

    def fly_mission_points(self, points):
        '''takes a list of waypoints and flies them, expecting a disarm at end'''
        self.check_mission_upload_download(points)
        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.set_rc(8, 1000)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("Raising rotor speed")
        self.set_rc(8, 2000)
        self.wait_waypoint(0, len(points)-1)
        self.wait_disarmed()
        self.set_rc(8, 1000)

    @testcase(bucket="Helicopter")
    def NastyMission(self):
        '''constructs and runs missions designed to test scurves'''
        self.fly_mission_points(self.scurve_nasty_mission())
        # hopefully we don't need this step forever:
        self.progress("Restting mission state machine by changing into LOITER")
        self.change_mode('LOITER')
        self.fly_mission_points(self.scurve_nasty_up_mission())
