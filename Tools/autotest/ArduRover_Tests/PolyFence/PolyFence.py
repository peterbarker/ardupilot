'''
Rover autotest: PolyFence

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PolyFence(TestGroup):
    def drive_somewhere_breach_boundary_and_rtl(self, loc, target_system=1, target_component=1, timeout=60):
        tstart = self.get_sim_time()
        seen_fence_breach = False

        type_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

        self.change_mode('GUIDED')

        # ModeGuided rejects targets outside the fence, so disable the fence
        # while sending the target and re-enable it once the autopilot has
        # accepted it.  Breach detection then runs against the saved target.
        self.do_fence_disable()
        self.mav.mav.set_position_target_global_int_send(
            0,
            target_system,
            target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask,
            int(loc.lat * 1.0e7),
            int(loc.lng * 1.0e7),
            0, # alt
            0, # x-ve
            0, # y-vel
            0, # z-vel
            0, # afx
            0, # afy
            0, # afz
            0, # yaw,
            0, # yaw-rate
        )
        self.do_fence_enable()

        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Did not breach boundary + RTL")
            m = self.mav.recv_match(blocking=True,
                                    timeout=1)
            if m is None:
                continue
            t = m.get_type()
            if t == "POSITION_TARGET_GLOBAL_INT":
                self.progress("Target: (%s)" % str(m))
            elif t == "GLOBAL_POSITION_INT":
                self.progress("Position: (%s)" % str(m))
            elif t == "FENCE_STATUS":
                self.progress("Fence: %s" % str(m))
                if m.breach_status != 0:
                    seen_fence_breach = True
                    self.progress("Fence breach detected!")
                    if m.breach_type != mavutil.mavlink.FENCE_BREACH_BOUNDARY:
                        raise NotAchievedException("Breach of unexpected type")
            if self.mode_is("RTL", cached=True) and seen_fence_breach:
                break
        self.wait_distance_to_home(3, 7, timeout=30)

    def wait_fence_not_breached(self, timeout=5):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise AutoTestTimeoutException("Fence remains breached")
            m = self.mav.recv_match(type='FENCE_STATUS',
                                    blocking=True,
                                    timeout=1)
            if m is None:
                self.progress("No FENCE_STATUS received")
                continue
            self.progress("STATUS: %s" % str(m))
            if m.breach_status == 0:
                break

    def test_poly_fence_noarms(self, target_system=1, target_component=1):
        '''various tests to ensure we can't arm when in breach of a polyfence'''
        self.start_subtest("Ensure PolyFence arming checks work")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                           target_system=target_system,
                           target_component=target_component)
        self.set_parameters({
            "FENCE_TYPE": 2,    # circle only
        })
        self.wait_fence_not_breached()
        # FIXME: should we allow this?
        self.progress("Ensure we can arm with no poly in place")
        self.change_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()
        self.set_parameters({
            "FENCE_TYPE": 6,    # polyfence + circle
        })

        self.test_poly_fence_noarms_exclusion_circle(target_system=target_system,
                                                     target_component=target_component)
        self.test_poly_fence_noarms_inclusion_circle(target_system=target_system,
                                                     target_component=target_component)
        self.test_poly_fence_noarms_exclusion_polyfence(target_system=target_system,
                                                        target_component=target_component)
        self.test_poly_fence_noarms_inclusion_polyfence(target_system=target_system,
                                                        target_component=target_component)

    def test_poly_fence_noarms_exclusion_circle(self, target_system=1, target_component=1):
        self.start_subtest("Ensure not armable when within an exclusion circle")

        here = self.get_location()

        items = [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                0, # current
                0, # autocontinue
                5, # p1 - radius
                0, # p2
                0, # p3
                0, # p4
                int(here.lat * 1e7), # latitude
                int(here.lng * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                0, # current
                0, # autocontinue
                5, # p1 - radius
                0, # p2
                0, # p3
                0, # p4
                int(self.offset_location_ne(here, 100, 100).lat * 1e7), # latitude
                int(here.lng * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)
        self.wait_message_field_values('FENCE_STATUS', {'breach_status': 1})
        try:
            self.arm_motors_with_rc_input()
        except NotAchievedException:
            pass
        if self.armed():
            raise NotAchievedException(
                "Armed when within exclusion zone")

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           [])
        self.wait_fence_not_breached()

    def test_poly_fence_noarms_inclusion_circle(self, target_system=1, target_component=1):
        self.start_subtest("Ensure not armable when outside an inclusion circle (but within another")

        here = self.get_location()

        items = [
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1 - radius
                0, # p2
                0, # p3
                0, # p4
                int(here.lat * 1e7), # latitude
                int(here.lng * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
                0, # current
                0, # autocontinue
                5, # p1 - radius
                0, # p2
                0, # p3
                0, # p4
                int(self.offset_location_ne(here, 100, 100).lat * 1e7), # latitude
                int(here.lng * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ]
        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)
        self.wait_message_field_values('FENCE_STATUS', {'breach_status': 1})
        try:
            self.arm_motors_with_rc_input()
        except NotAchievedException:
            pass
        if self.armed():
            raise NotAchievedException(
                "Armed when outside an inclusion zone")

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           [])
        self.wait_fence_not_breached()

    def test_poly_fence_noarms_exclusion_polyfence(self, target_system=1, target_component=1):
        self.start_subtest("Ensure not armable when inside an exclusion polyfence (but outside another")

        here = self.get_location()

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, -50, -50), # bl
                self.offset_location_ne(here, -50, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, -50), # tl,
            ]),
        ])
        self.wait_message_field_values('FENCE_STATUS', {'breach_status': 1})
        try:
            self.arm_motors_with_rc_input()
        except NotAchievedException:
            pass
        if self.armed():
            raise NotAchievedException(
                "Armed when within polygon exclusion zone")

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           [])
        self.wait_fence_not_breached()

    def test_poly_fence_noarms_inclusion_polyfence(self, target_system=1, target_component=1):
        self.start_subtest("Ensure not armable when outside an inclusion polyfence (but within another")

        here = self.get_location()

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, -50, -50), # bl
                self.offset_location_ne(here, -50, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, -50), # tl,
            ]),
        ])
        self.wait_message_field_values('FENCE_STATUS', {'breach_status': 1})
        try:
            self.arm_motors_with_rc_input()
        except NotAchievedException:
            pass
        if self.armed():
            raise NotAchievedException(
                "Armed when outside polygon inclusion zone")

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           [])
        self.wait_fence_not_breached()

    def test_fence_upload_timeouts_1(self, target_system=1, target_component=1):
        self.start_subtest("fence_upload timeouts 1")
        self.progress("Telling victim there will be one item coming")
        self.mav.mav.mission_count_send(target_system,
                                        target_component,
                                        1,
                                        mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        m = self.assert_receive_message(['MISSION_REQUEST', 'MISSION_ACK'], verbose=True)
        if m.get_type() == "MISSION_ACK":
            raise NotAchievedException("Expected MISSION_REQUEST")

        if m.seq != 0:
            raise NotAchievedException("Expected request for seq=0")

        if m.target_system != self.mav.mav.srcSystem:
            raise NotAchievedException("Incorrect target system in MISSION_REQUEST")
        if m.target_component != self.mav.mav.srcComponent:
            raise NotAchievedException("Incorrect target component in MISSION_REQUEST")
        tstart = self.get_sim_time()
        rerequest_count = 0
        received_text = False
        received_ack = False
        while True:
            if received_ack and received_text:
                break
            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException("Did not get expected ack and statustext")
            m = self.mav.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK', 'STATUSTEXT'],
                                    blocking=True,
                                    timeout=1)
            self.progress("Got (%s)" % str(m))
            if m is None:
                self.progress("Did not receive any messages")
                continue
            if m.get_type() == "MISSION_REQUEST":
                if m.seq != 0:
                    raise NotAchievedException("Received request for invalid seq")
                if m.target_system != self.mav.mav.srcSystem:
                    raise NotAchievedException("Incorrect target system in MISSION_REQUEST")
                if m.target_component != self.mav.mav.srcComponent:
                    raise NotAchievedException("Incorrect target component in MISSION_REQUEST")
                rerequest_count += 1
                self.progress("Valid re-request received.")
                continue
            if m.get_type() == "MISSION_ACK":
                if m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
                    raise NotAchievedException("Wrong mission type")
                if m.type != mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED:
                    raise NotAchievedException("Wrong result")
                received_ack = True
                continue
            if m.get_type() == "STATUSTEXT":
                if "upload time" in m.text:
                    received_text = True
                continue
        if rerequest_count < 3:
            raise NotAchievedException("Expected several re-requests of mission item")
        self.end_subtest("fence upload timeouts 1")

    def expect_request_for_item(self, item):
        m = self.assert_receive_message(['MISSION_REQUEST', 'MISSION_ACK'], verbose=-True)
        if m.get_type() == "MISSION_ACK":
            raise NotAchievedException("Expected MISSION_REQUEST")

        if m.seq != item.seq:
            raise NotAchievedException("Expected request for seq=%u" % item.seq)

        if m.target_system != self.mav.mav.srcSystem:
            raise NotAchievedException("Incorrect target system in MISSION_REQUEST")
        if m.target_component != self.mav.mav.srcComponent:
            raise NotAchievedException("Incorrect target component in MISSION_REQUEST")

    def test_fence_upload_timeouts_2(self, target_system=1, target_component=1):
        self.start_subtest("fence upload timeouts 2")
        self.progress("Telling victim there will be two items coming")
        # avoid a timeout race condition where ArduPilot re-requests a
        # fence point before we receive and respond to the first one.
        # Since ArduPilot has a 1s timeout on re-requesting, This only
        # requires a round-trip delay of 1/speedup seconds to trigger
        # - and that has been seen in practise on Travis
        self.context_push()
        self.context_set_speedup(1)
        self.mav.mav.mission_count_send(target_system,
                                        target_component,
                                        2,
                                        mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.progress("Sending item with seq=0")
        item = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            0, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            0, # current
            0, # autocontinue
            1, # p1 radius
            0, # p2
            0, # p3
            0, # p4
            int(1.1 * 1e7), # latitude
            int(1.2 * 1e7), # longitude
            33.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        self.expect_request_for_item(item)
        item.pack(self.mav.mav)
        self.mav.mav.send(item)

        self.progress("Sending item with seq=1")
        item = self.mav.mav.mission_item_int_encode(
            target_system,
            target_component,
            1, # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            0, # current
            0, # autocontinue
            1, # p1 radius
            0, # p2
            0, # p3
            0, # p4
            int(1.1 * 1e7), # latitude
            int(1.2 * 1e7), # longitude
            33.0000, # altitude
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)

        self.expect_request_for_item(item)

        self.context_pop()

        self.progress("Now waiting for a timeout")
        tstart = self.get_sim_time()
        rerequest_count = 0
        received_text = False
        received_ack = False
        while True:
            if received_ack and received_text:
                break
            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException("Did not get expected ack and statustext")
            m = self.mav.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK', 'STATUSTEXT'],
                                    blocking=True,
                                    timeout=0.1)
            self.progress("Got (%s)" % str(m))
            if m is None:
                self.progress("Did not receive any messages")
                continue
            if m.get_type() == "MISSION_REQUEST":
                if m.seq != 1:
                    raise NotAchievedException("Received request for invalid seq")
                if m.target_system != self.mav.mav.srcSystem:
                    raise NotAchievedException("Incorrect target system in MISSION_REQUEST")
                if m.target_component != self.mav.mav.srcComponent:
                    raise NotAchievedException("Incorrect target component in MISSION_REQUEST")
                rerequest_count += 1
                self.progress("Valid re-request received.")
                continue
            if m.get_type() == "MISSION_ACK":
                if m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
                    raise NotAchievedException("Wrong mission type")
                if m.type != mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED:
                    raise NotAchievedException("Wrong result")
                received_ack = True
                continue
            if m.get_type() == "STATUSTEXT":
                if "upload time" in m.text:
                    received_text = True
                continue
        if rerequest_count < 3:
            raise NotAchievedException("Expected several re-requests of mission item")
        self.end_subtest("fence upload timeouts 2")

    def test_fence_upload_timeouts(self, target_system=1, target_component=1):
        self.test_fence_upload_timeouts_1(target_system=target_system,
                                          target_component=target_component)
        self.test_fence_upload_timeouts_2(target_system=target_system,
                                          target_component=target_component)

    def test_poly_fence_reboot_survivability(self):
        here = self.get_location()

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, -50, -50), # bl
                self.offset_location_ne(here, -50, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, -50), # tl,
            ]),
        ])
        self.reboot_sitl()
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        downloaded_len = len(downloaded_items)
        if downloaded_len != 8:
            raise NotAchievedException("Items did not survive reboot (want=%u got=%u)" %
                                       (8, downloaded_len))

    @testcase(bucket="Rover")
    def PolyFence(self):
        '''test fence-related functions'''
        target_system = 1
        target_component = 1

        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        here = self.get_location()
        self.progress("here: %f %f" % (here.lat, here.lng))
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "AVOID_ENABLE": 0,
        })

#        self.set_parameter("SIM_SPEEDUP", 1)

        self.test_fence_upload_timeouts()

        self.test_poly_fence_noarms(target_system=target_system, target_component=target_component)

        self.arm_vehicle()

        self.test_poly_fence_inclusion(here, target_system=target_system, target_component=target_component)
        self.test_poly_fence_exclusion(here, target_system=target_system, target_component=target_component)

        self.disarm_vehicle()

        self.test_poly_fence_reboot_survivability()

    def test_poly_fence_inclusion_overlapping_inclusion_circles(self, here, target_system=1, target_component=1):
        self.start_subtest("Overlapping circular inclusion")
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, {
                "radius": 30,
                "loc": self.offset_location_ne(here, -20, 0),
            }),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, {
                "radius": 30,
                "loc": self.offset_location_ne(here, 20, 0),
            }),
        ])

        self.delay_sim_time(5, reason="fence to be uploaded")
        self.progress("Drive outside top circle")
        fence_middle = self.offset_location_ne(here, -150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

        self.progress("Drive outside bottom circle")
        fence_middle = self.offset_location_ne(here, 150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

    def test_poly_fence_inclusion(self, here, target_system=1, target_component=1):
        self.progress("Circle and Polygon inclusion")
        self.test_poly_fence_inclusion_overlapping_inclusion_circles(
            here,
            target_system=target_system,
            target_component=target_component)

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                self.offset_location_ne(here, -40, -20), # tl
                self.offset_location_ne(here, 50, -20), # tr
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, -40, 20), # bl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, {
                "radius": 30,
                "loc": self.offset_location_ne(here, -20, 0),
            }),
        ])

        self.progress("Drive outside polygon")
        fence_middle = self.offset_location_ne(here, -150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

        self.progress("Drive outside circle")
        fence_middle = self.offset_location_ne(here, 150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                self.offset_location_ne(here, -20, -25), # tl
                self.offset_location_ne(here, 50, -25), # tr
                self.offset_location_ne(here, 50, 15), # br
                self.offset_location_ne(here, -20, 15), # bl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                self.offset_location_ne(here, 20, -20), # tl
                self.offset_location_ne(here, -50, -20), # tr
                self.offset_location_ne(here, -50, 20), # br
                self.offset_location_ne(here, 20, 20), # bl,
            ]),
        ])

        self.progress("Drive outside top polygon")
        fence_middle = self.offset_location_ne(here, -150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

        self.wait_rtl_complete()

        self.progress("Drive outside bottom polygon")
        fence_middle = self.offset_location_ne(here, 150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(
            fence_middle,
            target_system=target_system,
            target_component=target_component)

    def test_poly_fence_exclusion(self, here, target_system=1, target_component=1):

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # west
                self.offset_location_ne(here, -50, -20), # tl
                self.offset_location_ne(here, 50, -20), # tr
                self.offset_location_ne(here, 50, -40), # br
                self.offset_location_ne(here, -50, -40), # bl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, {
                "radius": 30,
                "loc": self.offset_location_ne(here, -60, 0),
            }),
        ])

        self.progress("Breach eastern boundary")
        fence_middle = self.offset_location_ne(here, 0, 30)
        self.drive_somewhere_breach_boundary_and_rtl(fence_middle,
                                                     target_system=target_system,
                                                     target_component=target_component)

        self.progress("delaying - hack to work around manual recovery bug")
        self.delay_sim_time(5, reason="manual recovery from breach")

        self.progress("Breach western boundary")
        fence_middle = self.offset_location_ne(here, 0, -30)
        self.drive_somewhere_breach_boundary_and_rtl(fence_middle,
                                                     target_system=target_system,
                                                     target_component=target_component)

        self.progress("delaying - hack to work around manual recovery bug")
        self.delay_sim_time(5, reason="manual recovery from breach")

        self.progress("Breach southern circle")
        fence_middle = self.offset_location_ne(here, -150, 0)
        self.drive_somewhere_breach_boundary_and_rtl(fence_middle,
                                                     target_system=target_system,
                                                     target_component=target_component)
