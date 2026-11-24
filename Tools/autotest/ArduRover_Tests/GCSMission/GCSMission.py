'''
Rover autotest: GCSMission

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GCSMission(TestGroup):
    @testcase(bucket="Rover")
    def GCSMission(self):
        '''check MAVProxy's waypoint handling of missions'''
        # vehicle with state may have non-zero MISSION_CURRENT.seq, so reboot
        self.reboot_sitl()
        target_system = 1
        target_component = 1
        mavproxy = self.start_mavproxy()
        self.wait_parameter_value('MIS_TOTAL', 1)
        mavproxy.send('wp clear\n')
        self.wait_parameter_value('MIS_TOTAL', 0)
        self.assert_received_message_field_values('MISSION_CURRENT', {
            "seq": 0,
        }, timeout=5, verbose=True)
        self.load_mission_using_mavproxy(mavproxy, "rover-gripper-mission.txt")
        set_wp = 1
        mavproxy.send('wp set %u\n' % set_wp)
        self.wait_current_waypoint(set_wp)

        self.start_subsubtest("wp changealt")
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        changealt_item = 1
#        oldalt = downloaded_items[changealt_item].z
        want_newalt = 37.2
        mavproxy.send('wp changealt %u %f\n' % (changealt_item, want_newalt))
        self.delay_sim_time(15, reason="waypoint alt change to propagate")
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        if abs(downloaded_items[changealt_item].z - want_newalt) > 0.0001:
            raise NotAchievedException(
                "changealt didn't (want=%f got=%f)" %
                (want_newalt, downloaded_items[changealt_item].z))
        self.end_subsubtest("wp changealt")

        self.start_subsubtest("wp sethome")
        new_home_lat = 3.14159
        new_home_lng = 2.71828
        mavproxy.send('click %f %f\n' % (new_home_lat, new_home_lng))
        mavproxy.send('wp sethome\n')
        self.delay_sim_time(5, reason="home waypoint to be set")
        # any way to close the loop on this one?
        # downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        # if abs(downloaded_items[0].x - new_home_lat) > 0.0001:
        #     raise NotAchievedException("wp sethome didn't work")
        # if abs(downloaded_items[0].y - new_home_lng) > 0.0001:
        #     raise NotAchievedException("wp sethome didn't work")
        self.end_subsubtest("wp sethome")

        self.start_subsubtest("wp slope")
        mavproxy.send('wp slope\n')
        mavproxy.expect("WP3: slope 0.1")
        self.end_subsubtest("wp slope")

        if not self.mavproxy_can_do_mision_item_protocols():
            # adding based on click location yet to be merged into MAVProxy
            return

        self.start_subsubtest("wp split")
        mavproxy.send("wp clear\n")
        self.delay_sim_time(5, reason="waypoint clear to propagate")
        mavproxy.send("wp list\n")
        self.delay_sim_time(5, reason="waypoint list to display")
        items = [
            None,
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0 * 1e7), # latitude
                int(1.0 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(2.0 * 1e7), # latitude
                int(2.0 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
        ]
        mavproxy.send("click 5 5\n") # space for home position
        mavproxy.send("wp add\n")
        self.delay_sim_time(5, reason="waypoint add to propagate")
        self.click_location_from_item(mavproxy, items[1])
        mavproxy.send("wp add\n")
        self.delay_sim_time(5, reason="waypoint add to propagate")
        self.click_location_from_item(mavproxy, items[2])
        mavproxy.send("wp add\n")
        self.delay_sim_time(5, reason="waypoint add to propagate")
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        self.check_mission_waypoint_items_same(items, downloaded_items)
        mavproxy.send("wp split 2\n")
        self.delay_sim_time(5, reason="waypoint split to propagate")
        items_with_split_in = [
            items[0],
            items[1],
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.5 * 1e7), # latitude
                int(1.5 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            items[2],
        ]
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        self.check_mission_waypoint_items_same(items_with_split_in,
                                               downloaded_items)

        mavproxy.send('wp clear\n')
        self.wait_parameter_value('MIS_TOTAL', 0)

        self.stop_mavproxy(mavproxy)

        # MANUAL> usage: wp <changealt|clear|draw|editor|list|load|loop|move|movemulti|noflyadd|param|remove|save|savecsv|savelocal|set|sethome|show|slope|split|status|undo|update>  # noqa
