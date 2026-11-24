'''
Copter autotest: MAVLinkCameraRelay

AP_FLAKE8_CLEAN
'''

import time

from pymavlink import mavutil

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAVLinkCameraRelay(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MAVLinkCameraRelay(self):
        '''relay isolated camera broadcasts without losing camera identity'''
        self.start_subtest("Default camera component IDs")
        self._test_mavlink_camera_relay((0, 0))
        self.start_subtest("Configured camera component IDs, including IDs above 127")
        self._test_mavlink_camera_relay((200, 255))

    def _test_mavlink_camera_relay(self, component_ids):
        self.set_parameters({
            "CAM1_TYPE": 6,
            "CAM2_TYPE": 6,
            "MNT1_TYPE": 1,
            "MNT2_TYPE": 1,
            "CAM1_COMPID": component_ids[0],
            "CAM2_COMPID": component_ids[1],
            "SERIAL1_PROTOCOL": 2,
            "SERIAL2_PROTOCOL": 2,
            "SERIAL5_PROTOCOL": 2,
            "SERIAL6_PROTOCOL": 2,
        })
        self.reboot_sitl()
        self.set_parameters({"MAV2_OPTIONS": 0, "MAV3_OPTIONS": 16, "MAV4_OPTIONS": 2, "MAV5_OPTIONS": 16})
        self.reboot_sitl()
        links = {"gcs": self.mav}
        saved_mavfile_global = mavutil.mavfile_global
        camera_compid, private_camera_compid = (
            compid or mavutil.mavlink.MAV_COMP_ID_CAMERA + instance
            for instance, compid in enumerate(component_ids))
        try:
            for name, serial, compid in (("normal", 1, 191), ("camera", 2, camera_compid),
                                         ("private_camera", 5, private_camera_compid), ("device", 6, 102)):
                links[name] = mavutil.mavlink_connection(
                    self.sitl_serial_endpoint(serial), source_system=self.sysid_thismav() + int(name == "private_camera"),
                    source_component=compid,
                    robust_parsing=True)
            mavutil.mavfile_global = saved_mavfile_global

            def collect(duration=1):
                received = {name: [] for name in links}
                start = self.get_sim_time_cached()
                wall_start = time.time()
                while self.get_sim_time_cached() - start < duration:
                    if time.time() - wall_start > 10:
                        raise AutoTestTimeoutException("Simulation stopped during camera relay check")
                    for name, link in links.items():
                        while (msg := link.recv_match()) is not None:
                            received[name].append(msg)
                    self.mav.select(0.01)
                return received

            for name, link in links.items():
                mav_type = (mavutil.mavlink.MAV_TYPE_CAMERA if name in ("camera", "private_camera") else
                            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER)
                link.mav.heartbeat_send(mav_type, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            collect(12)  # camera backends start looking for routes after 10s
            for name in ("camera", "private_camera"):
                links[name].mav.camera_information_send(
                    0, b"ArduPilot".ljust(32, b'\0'), b"RelayTest".ljust(32, b'\0'),
                    1, 0, 0, 0, 640, 480, 0, 0, 0, b"")
            collect()

            def check_relay(source, message, expected, bad_crc=False):
                collect(0.2)
                if bad_crc:
                    wire = bytearray(message.pack(links[source].mav))
                    wire[-1] ^= 1
                    links[source].write(wire)
                else:
                    links[source].mav.send(message)
                    wire = bytes(message.get_msgbuf())
                received = collect()
                for name, messages in received.items():
                    replies = [msg for msg in messages if msg.get_type() == message.get_type() and
                               (msg.get_type() != 'HEARTBEAT' or msg.type == mavutil.mavlink.MAV_TYPE_CAMERA)]
                    want = int(name in expected)
                    if len(replies) != want or any(bytes(reply.get_msgbuf()) != wire for reply in replies):
                        raise NotAchievedException("Camera relay %s from %s to %s: got %s, want %u unchanged packets" %
                                                   (message.get_type(), source, name, replies, want))

            replies = (
                mavutil.mavlink.MAVLink_param_ext_value_message(
                    b"CAM_TEST", b"7", mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT32, 1, 0),
                mavutil.mavlink.MAVLink_param_ext_ack_message(
                    b"CAM_TEST", b"7", mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT32, mavutil.mavlink.PARAM_ACK_ACCEPTED),
                mavutil.mavlink.MAVLink_camera_thermal_range_message(123, 1, 0, 80, 0.5, 0.5, 20, 0.1, 0.1),
                mavutil.mavlink.MAVLink_camera_tracking_image_status_message(1, 1, 1, 0.5, 0.5, 0.1, 0, 0, 0, 0),
                mavutil.mavlink.MAVLink_camera_tracking_geo_status_message(1, 123456, 234567, 80, 1, 1, 0, 0, 0, 1, 10, 0, 1),
                mavutil.mavlink.MAVLink_video_stream_status_message(1, 5, 30, 640, 480, 1000000, 0, 60),
                mavutil.mavlink.MAVLink_heartbeat_message(
                    mavutil.mavlink.MAV_TYPE_CAMERA, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3),
                mavutil.mavlink.MAVLink_camera_information_message(
                    123, b"ArduPilot".ljust(32, b'\0'), b"RelayTest".ljust(32, b'\0'),
                    1, 0, 0, 0, 640, 480, 0, 0, 1, b"mftp://[;comp=200]/camera.xml", 154),
                mavutil.mavlink.MAVLink_camera_settings_message(123, 1, 42, 17),
                mavutil.mavlink.MAVLink_storage_information_message(123, 1, 1, 2, 100, 20, 80, 0, 0),
                mavutil.mavlink.MAVLink_camera_capture_status_message(123, 1, 1, 0, 456, 80, 7),
                mavutil.mavlink.MAVLink_camera_image_captured_message(
                    123, 0, 0, 0, 0, 0, 0, [1, 0, 0, 0], 7, 1, b"image.jpg"),
                mavutil.mavlink.MAVLink_camera_fov_status_message(
                    123, 0, 0, 0, 0, 0, 0, [1, 0, 0, 0], 60, 45),
                mavutil.mavlink.MAVLink_video_stream_information_message(
                    1, 1, mavutil.mavlink.VIDEO_STREAM_TYPE_RTSP, 1, 30, 640, 480, 1000000, 0, 60,
                    b"Visible", b"rtsp://camera/video1"),
            )
            self.progress("Checking two isolated cameras retain distinct identities and cannot reach each other")
            for source in ("camera", "private_camera"):
                for message in replies:
                    check_relay(source, message, ["gcs", "normal"])
            self.set_parameter("MAV4_OPTIONS", 18)
            for message in replies:
                check_relay("private_camera", message, ["gcs", "normal"])

            self.progress("Checking cached replies retain both native camera identities")
            for message_type in ('CAMERA_INFORMATION', 'VIDEO_STREAM_INFORMATION', 'CAMERA_CAPTURE_STATUS'):
                payload = next(msg for msg in replies if msg.get_type() == message_type)
                for source in ("camera", "private_camera"):
                    links[source].mav.send(payload)
                # Drain the unsolicited relays before collecting cached
                # replies, while staying below the 3s capture-status expiry.
                collect(1)
                self.send_poll_message(message_type)
                received = [msg for msg in collect()["gcs"] if msg.get_type() == message_type]
                identities = [(msg.get_srcSystem(), msg.get_srcComponent()) for msg in received]
                if sorted(identities) != [(self.sysid_thismav(), camera_compid),
                                          (self.sysid_thismav() + 1, private_camera_compid)]:
                    raise NotAchievedException("Cached %s identities: %s" % (message_type, identities))
                for msg in received:
                    # Compare MAVLink 2 payloads, excluding regenerated headers/checksums.
                    if bytes(msg.get_msgbuf())[10:-2] != bytes(payload.get_msgbuf())[10:-2]:
                        raise NotAchievedException("Cached %s changed camera metadata: %s" % (message_type, msg))
            for message_type in ('CAMERA_SETTINGS', 'CAMERA_FOV_STATUS'):
                self.send_poll_message(message_type)
                if any(msg.get_type() == message_type for msg in collect()["gcs"]):
                    raise NotAchievedException("Native cameras produced synthetic %s" % message_type)

            self.progress("Checking a camera in another system cannot inherit this FC's mount")
            links['private_camera'].mav.camera_information_send(
                123, b"ArduPilot".ljust(32, b'\0'), b"Standalone".ljust(32, b'\0'),
                1, 0, 0, 0, 640, 480, 0, 0, 0, b"", 0)
            collect(1)
            self.send_poll_message('CAMERA_INFORMATION')
            foreign = [msg for msg in collect()['gcs'] if msg.get_type() == 'CAMERA_INFORMATION' and
                       msg.get_srcSystem() == self.sysid_thismav() + 1]
            if len(foreign) != 1 or foreign[0].gimbal_device_id != 0:
                raise NotAchievedException("Foreign camera inherited an FC mount: %s" % foreign)

            self.progress("Checking unconfigured devices and mismatched camera links are not relayed")
            for message in replies:
                check_relay("device", message, [])
            links["camera"].mav.srcComponent = 103
            check_relay("camera", replies[2], [])
            links["camera"].mav.srcComponent = camera_compid
            links["camera"].mav.srcSystem = self.sysid_thismav() + 1
            check_relay("camera", replies[2], [])
            links["camera"].mav.srcSystem = self.sysid_thismav()

            self.progress("Checking camera relaying does not duplicate ordinary broadcast forwarding")
            self.set_parameter("MAV3_OPTIONS", 0)
            for message in replies:
                check_relay("camera", message, ["gcs", "normal"])
                check_relay("normal", message, ["gcs", "camera"])
            self.set_parameter("MAV3_OPTIONS", 16)

            self.progress("Checking other camera broadcasts and bad-CRC messages remain isolated")
            check_relay("camera", mavutil.mavlink.MAVLink_named_value_int_message(123, b"CAM_RELAY", 42), [])
            self.set_parameter("MAV3_OPTIONS", 24)
            for message in replies:
                check_relay("camera", message, [], bad_crc=True)
            self.set_parameter("MAV3_OPTIONS", 16)

            self.progress("Checking addressed tracking requests and acknowledgements still route unchanged")
            for command in (mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT,
                            mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE,
                            mavutil.mavlink.MAV_CMD_CAMERA_STOP_TRACKING):
                check_relay("gcs", mavutil.mavlink.MAVLink_command_long_message(
                    self.sysid_thismav(), camera_compid, command, 0, 0.1, 0.2, 0.3, 0.4, 0, 0, 0), ["camera"])
                check_relay("camera", mavutil.mavlink.MAVLink_command_ack_message(
                    command, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0, 0,
                    self.mav.mav.srcSystem, self.mav.mav.srcComponent), ["gcs"])

            for source in ("camera", "private_camera"):
                sysid = links[source].mav.srcSystem
                compid = links[source].mav.srcComponent
                for request in (
                        mavutil.mavlink.MAVLink_param_ext_request_list_message(sysid, compid),
                        mavutil.mavlink.MAVLink_param_ext_request_read_message(
                            sysid, compid, b"CAM_TEST", -1),
                        mavutil.mavlink.MAVLink_param_ext_set_message(
                            sysid, compid, b"CAM_TEST", b"8", mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT32),
                        mavutil.mavlink.MAVLink_command_long_message(
                            sysid, compid, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                            0, mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_INFORMATION, 0, 0, 0, 0, 0, 0),
                        mavutil.mavlink.MAVLink_command_long_message(
                            sysid, compid, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                            0, mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE, 200000, 1, 0, 0, 0, 0)):
                    check_relay("gcs", request, [source])

            # This also learns an extra route, so do it after addressed routing checks.
            links["device"].mav.srcComponent = camera_compid
            check_relay("device", replies[2], [])  # right sysid/compid, wrong link
        finally:
            mavutil.mavfile_global = saved_mavfile_global
            for name, link in links.items():
                if name != "gcs":
                    link.close()
