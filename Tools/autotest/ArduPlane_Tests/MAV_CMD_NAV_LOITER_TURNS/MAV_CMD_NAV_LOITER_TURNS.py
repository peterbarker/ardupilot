'''
Plane autotest: MAV_CMD_NAV_LOITER_TURNS

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_LOITER_TURNS(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def MAV_CMD_NAV_LOITER_TURNS(self, target_system=1, target_component=1):
        '''test MAV_CMD_NAV_LOITER_TURNS mission item'''
        alt = 100
        seq = 0
        items = []
        home = self.home_position_as_location()
        tests = [
            (self.offset_location_ne(home, 50, -50), 100, 0.3),
            (self.offset_location_ne(home, 100, 50), 1005, 3),
        ]
        # add a home position:
        items.append(self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            seq, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, # current
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            0, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION))
        seq += 1

        # add takeoff
        items.append(self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            seq, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, # current
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            alt, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION))
        seq += 1

        # add circles
        for (loc, radius, turn) in tests:
            items.append(self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                seq, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                0, # current
                0, # autocontinue
                turn, # p1
                0, # p2
                radius, # p3
                0, # p4
                int(loc.lat*1e7), # latitude
                int(loc.lng*1e7), # longitude
                alt, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION))
            seq += 1

        # add an RTL
        items.append(self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            seq, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, # current
            0, # autocontinue
            0, # p1
            0, # p2
            0, # p3
            0, # p4
            0, # latitude
            0, # longitude
            0, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION))
        seq += 1

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION, items)
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        ofs = 2
        self.progress("Checking downloaded mission is as expected")
        for (loc, radius, turn) in tests:
            downloaded = downloaded_items[ofs]
            if radius > 255:
                # ArduPilot only stores % 10
                radius = radius - radius % 10
            if downloaded.param3 != radius:
                raise NotAchievedException(
                    "Did not get expected radius for item %u; want=%f got=%f" %
                    (ofs, radius, downloaded.param3))
            if turn > 0 and turn < 1:
                # ArduPilot stores fractions in 8 bits (*256) and unpacks it (/256)
                turn = int(turn*256) / 256.0
            if downloaded.param1 != turn:
                raise NotAchievedException(
                    "Did not get expected turn for item %u; want=%f got=%f" %
                    (ofs, turn, downloaded.param1))
            ofs += 1

        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter("NAVL1_LIM_BANK", 50)

        self.wait_current_waypoint(2)

        for (loc, expected_radius, _) in tests:
            self.wait_circling_point_with_radius(
                loc,
                expected_radius,
                epsilon=20.0,
                timeout=240,
            )
            self.set_current_waypoint(self.current_waypoint()+1)

        self.fly_home_land_and_disarm(timeout=180)
