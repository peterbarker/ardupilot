'''
Copter autotest: MAVLinkCameraComponentIDs

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAVLinkCameraComponentIDs(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MAVLinkCameraComponentIDs(self):
        '''reject invalid or colliding camera component IDs without duplicate relays'''
        for message_type in ('STATUSTEXT', 'HEARTBEAT', 'CAMERA_INFORMATION'):
            self.context_collect(message_type)
        cases = ((-1, 0, 255, False), (256, 0, 100, False), (1, 0, 1, False), (6, 0, 6, False),
                 (101, 0, 101, True), (0, 100, 100, True), (200, 200, 200, True))
        for first, second, source_compid, duplicate in cases:
            self.start_subtest("Camera component IDs %u/%u" % (first, second))
            self.set_parameters({
                "CAM1_TYPE": 6,
                "CAM2_TYPE": 6 if duplicate else 0,
                "CAM1_COMPID": first,
                "CAM2_COMPID": second,
                "SERIAL1_PROTOCOL": 2,
                "SERIAL2_PROTOCOL": 2,
                "MAV3_OPTIONS": 16,
            })
            self.context_clear_collection('STATUSTEXT')
            self.reboot_sitl()
            saved_mavfile_global = mavutil.mavfile_global
            camera = mavutil.mavlink_connection(
                self.sitl_serial_endpoint(2), source_system=self.sysid_thismav(), source_component=source_compid)
            mavutil.mavfile_global = saved_mavfile_global
            try:
                camera.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_CAMERA, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self.wait_statustext("CAM2_COMPID duplicates CAM1" if duplicate else "CAM1_COMPID must be 0 or 7..255",
                                     check_context=True)
                if duplicate:
                    # The backend only relays once it has found the camera in
                    # the routing table on a later update, so a message sent
                    # straight after the heartbeat is dropped. Its first act on
                    # discovery is to request CAMERA_INFORMATION; wait for that.
                    tstart = self.get_sim_time()
                    while True:
                        if self.get_sim_time_cached() - tstart > 10:
                            raise NotAchievedException("Configured camera was not discovered")
                        request = camera.recv_match(type='COMMAND_LONG', blocking=False)
                        if (request is not None and
                                request.target_component == source_compid and
                                request.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE and
                                int(request.param1) == mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_INFORMATION):
                            break
                        self.delay_sim_time(0.2, reason="wait for camera discovery")
                self.context_clear_collection('HEARTBEAT')
                self.context_clear_collection('CAMERA_INFORMATION')
                camera.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_CAMERA, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                camera.mav.camera_information_send(
                    123, b"ArduPilot".ljust(32, b'\0'), b"ComponentTest".ljust(32, b'\0'),
                    1, 0, 0, 0, 640, 480, 0, 0, 0, b"")
                self.delay_sim_time(1, reason="collect configured camera relays")
                for message_type in ('HEARTBEAT', 'CAMERA_INFORMATION'):
                    messages = [m for m in self.context_collection(message_type) if m.get_srcComponent() == source_compid and
                                (message_type != 'HEARTBEAT' or m.type == mavutil.mavlink.MAV_TYPE_CAMERA)]
                    if len(messages) != int(duplicate):
                        raise NotAchievedException("Misconfigured camera relays %s: %s" % (message_type, messages))
                self.context_clear_collection('CAMERA_INFORMATION')
                self.send_poll_message('CAMERA_INFORMATION')
                self.delay_sim_time(0.5, reason="collect configured camera cache replies")
                messages = self.context_collection('CAMERA_INFORMATION')
                if len(messages) != int(duplicate):
                    raise NotAchievedException("Misconfigured camera cache replies: %s" % messages)
            finally:
                camera.close()
