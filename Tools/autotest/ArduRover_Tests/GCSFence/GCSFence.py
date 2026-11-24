'''
Rover autotest: GCSFence

AP_FLAKE8_CLEAN
'''

import copy

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GCSFence(TestGroup):
    def fence_with_bad_frame(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_zero_vertex_count(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_wrong_vertex_count(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                2, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_multiple_return_points(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_invalid_latlon(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(100 * 1e7), # bad latitude. bad.
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_multiple_return_points_with_bad_sequence_numbers(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(2.0 * 1e7), # latitude
                int(2.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_which_exceeds_storage_space(self, target_system=1, target_component=1):
        ret = []
        for i in range(0, 60):
            ret.append(self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                i, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                0, # current
                0, # autocontinue
                10, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            )
        return ret

    def fences_which_should_not_upload(self, target_system=1, target_component=1):
        return [
            ("Bad Frame", self.fence_with_bad_frame(
                target_system=target_system,
                target_component=target_component)),
            ("Zero Vertex Count", self.fence_with_zero_vertex_count(
                target_system=target_system,
                target_component=target_component)),
            ("Wrong Vertex Count", self.fence_with_wrong_vertex_count(
                target_system=target_system,
                target_component=target_component)),
            ("Multiple return points", self.fence_with_multiple_return_points(
                target_system=target_system,
                target_component=target_component)),
            ("Invalid lat/lon", self.fence_with_invalid_latlon(
                target_system=target_system,
                target_component=target_component)),
            ("Multiple Return points with bad sequence numbers",
                 self.fence_with_multiple_return_points_with_bad_sequence_numbers(  # noqa
                     target_system=target_system,
                     target_component=target_component)),
            ("Fence which exceeds storage space",
             self.fence_which_exceeds_storage_space(
                 target_system=target_system,
                 target_component=target_component)),
        ]

    def fence_with_single_return_point(self, target_system=1, target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_single_return_point_and_5_vertex_inclusion(self,
                                                              target_system=1,
                                                              target_component=1):
        return [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0017 * 1e7), # latitude
                int(1.0017 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0001 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                32.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                3, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0001 * 1e7), # latitude
                int(1.0001 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                4, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0002 * 1e7), # latitude
                int(1.0002 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                5, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0002 * 1e7), # latitude
                int(1.0003 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]

    def fence_with_many_exclusion_circles(self, count=50, target_system=1, target_component=1):
        ret = []
        for i in range(0, count):
            lat_deg = 1.0003 + count/10
            lng_deg = 1.0002 + count/10
            item = self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                i, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                0, # current
                0, # autocontinue
                count, # p1
                0, # p2
                0, # p3
                0, # p4
                int(lat_deg * 1e7), # latitude
                int(lng_deg * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
            ret.append(item)
        return ret

    def fence_with_many_exclusion_polyfences(self, target_system=1, target_component=1):
        ret = []
        seq = 0
        for fencenum in range(0, 4):
            pointcount = fencenum + 6
            for p in range(0, pointcount):
                lat_deg = 1.0003 + p/10 + fencenum/100
                lng_deg = 1.0002 + p/10 + fencenum/100
                item = self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    seq, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
                    0, # current
                    0, # autocontinue
                    pointcount, # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    int(lat_deg * 1e7), # latitude
                    int(lng_deg * 1e7), # longitude
                    33.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
                ret.append(item)
                seq += 1
        return ret

    def fences_which_should_upload(self, target_system=1, target_component=1):
        return [
            ("Single Return Point",
             self.fence_with_single_return_point(
                 target_system=target_system,
                 target_component=target_component)),
            ("Return and 5-vertex-inclusion",
             self.fence_with_single_return_point_and_5_vertex_inclusion(
                 target_system=target_system,
                 target_component=target_component)),
            ("Many exclusion circles",
             self.fence_with_many_exclusion_circles(
                 target_system=target_system,
                 target_component=target_component)),
            ("Many exclusion polyfences",
             self.fence_with_many_exclusion_polyfences(
                 target_system=target_system,
                 target_component=target_component)),
            ("Empty fence", []),
        ]

    def assert_fence_does_not_upload(self, fence, target_system=1, target_component=1):
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)
        # upload single item using mission item protocol:
        upload_failed = False
        try:
            self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                               fence)
        except NotAchievedException:
            # TODO: make sure we failed for correct reason
            upload_failed = True
        if not upload_failed:
            raise NotAchievedException("Uploaded fence when should not be possible")
        self.progress("Fence rightfully bounced")

    def test_gcs_fence_update_fencepoint(self, target_system=1, target_component=1):
        self.start_subtest("Ensuring we can move a fencepoint")
        items = self.test_gcs_fence_boring_triangle(
            target_system=target_system,
            target_component=target_component)
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)
#        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        item_seq = 2
        item = items[item_seq]
        print("item is (%s)" % str(item))
        self.progress("original x=%d" % item.x)
        item.x += int(0.1 * 1e7)
        self.progress("new x=%d" % item.x)
        self.progress("try to overwrite item %u" % item_seq)
        self.mav.mav.mission_write_partial_list_send(
            target_system,
            target_component,
            item_seq,
            item_seq,
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_FENCE, item_seq)
        item.pack(self.mav.mav)
        self.mav.mav.send(item)
        self.progress("Answered request for fence point %u" % item_seq)

        self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        downloaded_items2 = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if downloaded_items2[item_seq].x != item.x:
            raise NotAchievedException("Item did not update")
        self.check_fence_items_same([items[0], items[1], item, items[3]], downloaded_items2)

    def test_gcs_fence_boring_triangle(self, target_system=1, target_component=1):
        return copy.copy([
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0001 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                32.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                3, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0001 * 1e7), # latitude
                int(1.0001 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                3, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.00015 * 1e7), # latitude
                int(1.00015 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ])

    def test_gcs_fence_via_mavproxy(self, target_system=1, target_component=1):
        self.start_subtest("Fence via MAVProxy")
        if not self.mavproxy_can_do_mision_item_protocols():
            return
        mavproxy = self.start_mavproxy()
        self.start_subsubtest("fence addcircle")
        self.clear_fence_using_mavproxy(mavproxy)
        self.delay_sim_time(1, reason="fence clear to propagate")
        radius = 20
        item = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
            0, # current
            0, # autocontinue
            radius, # p1
            0, # p2
            0, # p3
            0, # p4
            int(1.0017 * 1e7), # latitude
            int(1.0017 * 1e7), # longitude
            0.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        print("item is (%s)" % str(item))
        self.click_location_from_item(mavproxy, item)
        mavproxy.send("fence addcircle inc %u\n" % radius)
        self.delay_sim_time(1, reason="fence upload to complete")
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        print("downloaded items: %s" % str(downloaded_items))
        self.check_fence_items_same([item], downloaded_items)

        radius_exc = 57.3
        item2 = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            0, # current
            0, # autocontinue
            radius_exc, # p1
            0, # p2
            0, # p3
            0, # p4
            int(1.0017 * 1e7), # latitude
            int(1.0017 * 1e7), # longitude
            0.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.click_location_from_item(mavproxy, item2)
        mavproxy.send("fence addcircle exc %f\n" % radius_exc)
        self.delay_sim_time(1, reason="fence upload to complete")
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        print("downloaded items: %s" % str(downloaded_items))
        self.check_fence_items_same([item, item2], downloaded_items)
        self.end_subsubtest("fence addcircle")

        self.start_subsubtest("fence addpoly")
        self.clear_fence_using_mavproxy(mavproxy)
        self.delay_sim_time(1, reason="fence clear to propagate")
        pointcount = 7
        mavproxy.send("fence addpoly inc 20 %u 37.2\n" % pointcount) # radius, pointcount, rotation
        self.delay_sim_time(5, reason="fence upload to complete")
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if len(downloaded_items) != pointcount:
            raise NotAchievedException("Did not get expected number of points returned (want=%u got=%u)" %
                                       (pointcount, len(downloaded_items)))
        self.end_subsubtest("fence addpoly")

        self.start_subsubtest("fence movepolypoint")
        self.clear_fence_using_mavproxy(mavproxy)
        self.delay_sim_time(1, reason="fence clear to propagate")
        triangle = self.test_gcs_fence_boring_triangle(
            target_system=target_system,
            target_component=target_component)
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           triangle)
        mavproxy.send("fence list\n")
        self.delay_sim_time(1, reason="fence list to display in MAVProxy")
        triangle[2].x += 500
        triangle[2].y += 700
        self.click_location_from_item(mavproxy, triangle[2])
        mavproxy.send("fence movepolypoint 0 2\n")
        self.delay_sim_time(10, reason="fence point move to propagate")
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.check_fence_items_same(triangle, downloaded_items)
        self.end_subsubtest("fence movepolypoint")

        self.start_subsubtest("fence enable and disable")
        mavproxy.send("fence enable\n")
        mavproxy.expect("fence enabled")
        mavproxy.send("fence disable\n")
        mavproxy.expect("fence disabled")
        self.end_subsubtest("fence enable and disable")

        self.stop_mavproxy(mavproxy)

    @testcase(bucket="Rover")
    def GCSFence(self):
        '''Upload and download of fence'''
        target_system = 1
        target_component = 1

        if not self.mavproxy_can_do_mision_item_protocols():
            self.progress("MAVProxy too old to do fence point protocols")
            return

        # upload using mission protocol:
        items = self.test_gcs_fence_boring_triangle(
            target_system=target_system,
            target_component=target_component)
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)

        self.progress("Download with new protocol")
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        if len(downloaded_items) != len(items):
            raise NotAchievedException("Did not download expected number of items (wanted=%u got=%u)" %
                                       (len(items), len(downloaded_items)))
        self.assert_parameter_value("FENCE_TOTAL", len(items) + 1)  # +1 for closing
        self.progress("Ensuring fence items match what we sent up")
        self.check_fence_items_same(items, downloaded_items)

        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)
        self.progress("Checking count post-nuke")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)
        self.assert_mission_count_on_link(self.mav,
                                          0,
                                          target_system,
                                          target_component,
                                          mavutil.mavlink.MAV_MISSION_TYPE_FENCE)

        self.start_subtest("Ensuring bad fences get bounced")
        for fence in self.fences_which_should_not_upload(target_system=target_system, target_component=target_component):
            (name, items) = fence
            self.progress("Ensuring (%s) gets bounced" % (name,))
            self.assert_fence_does_not_upload(items)

        self.start_subtest("Ensuring good fences don't get bounced")
        for fence in self.fences_which_should_upload(target_system=target_system, target_component=target_component):
            (name, items) = fence
            self.progress("Ensuring (%s) gets uploaded" % (name,))
            self.check_fence_upload_download(items)
            self.progress("(%s) uploaded just fine" % (name,))

        self.test_gcs_fence_update_fencepoint(target_system=target_system,
                                              target_component=target_component)

        self.test_gcs_fence_via_mavproxy(target_system=target_system,
                                         target_component=target_component)
