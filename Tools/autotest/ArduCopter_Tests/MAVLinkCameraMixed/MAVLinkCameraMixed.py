'''
Copter autotest: MAVLinkCameraMixed

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAVLinkCameraMixed(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MAVLinkCameraMixed(self):
        '''a native camera and an FC-owned camera must remain separate endpoints'''
        self.set_parameters({
            "CAM1_TYPE": 6,
            "CAM2_TYPE": 1,
            "MNT1_TYPE": 1,
            "MNT2_TYPE": 1,
            "CAM1_MNT_INST": 2,
            "SERIAL1_PROTOCOL": 2,
            "SERIAL2_PROTOCOL": 2,
            "MAV3_OPTIONS": 16,
        })
        self.reboot_sitl()
        saved_mavfile_global = mavutil.mavfile_global
        camera = mavutil.mavlink_connection(
            self.sitl_serial_endpoint(2), source_system=self.sysid_thismav(),
            source_component=mavutil.mavlink.MAV_COMP_ID_CAMERA)
        mavutil.mavfile_global = saved_mavfile_global
        try:
            self.context_collect('COMMAND_ACK')
            for run_cmd in (self.run_cmd, self.run_cmd_int):
                self.context_clear_collection('COMMAND_ACK')
                run_cmd(mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE, p1=2, p3=1)
                acks = [m for m in self.context_collection('COMMAND_ACK')
                        if m.command == mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE]
                if not acks or acks[-1].result_param2 != 2:
                    raise NotAchievedException("FC camera ACK lost its camera ID: %s" % acks)
                self.delay_sim_time(1, reason="collect selector replies")
            message_types = ('HEARTBEAT', 'CAMERA_INFORMATION', 'CAMERA_SETTINGS', 'CAMERA_CAPTURE_STATUS')
            for message_type in message_types:
                self.context_collect(message_type)
            camera.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_CAMERA, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.delay_sim_time(12, reason="discover mixed native camera")

            for options in (16, 0):
                self.set_parameter("MAV3_OPTIONS", options)
                self.context_clear_collection('HEARTBEAT')
                camera.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_CAMERA, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self.delay_sim_time(0.5, reason="collect native camera heartbeat")
                native = [m for m in self.context_collection('HEARTBEAT') if m.type == mavutil.mavlink.MAV_TYPE_CAMERA]
                if len(native) != 1 or native[0].get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_CAMERA:
                    raise NotAchievedException("Mixed camera discovery lost or duplicated the native endpoint")

                camera.mav.camera_information_send(
                    123, b"ArduPilot".ljust(32, b'\0'), b"Native".ljust(32, b'\0'),
                    1, 0, 0, 0, 640, 480, 0, 0, 0, b"", 154)
                camera.mav.camera_capture_status_send(123, 0, 1, 0, 456, 80, 7)
                self.delay_sim_time(0.2, reason="cache native camera metadata")

                for message_type in ('CAMERA_INFORMATION', 'CAMERA_CAPTURE_STATUS', 'CAMERA_SETTINGS'):
                    self.context_clear_collection(message_type)
                    self.send_poll_message(message_type)
                    self.delay_sim_time(0.5, reason="collect mixed camera replies")
                    messages = self.context_collection(message_type)
                    expected = [1] if message_type == 'CAMERA_SETTINGS' else [1, 100]
                    if sorted(m.get_srcComponent() for m in messages) != expected:
                        raise NotAchievedException("Mixed %s identities: %s" % (message_type, messages))
                    if any(m.get_srcSystem() != self.sysid_thismav() for m in messages):
                        raise NotAchievedException("Mixed camera system ID changed")
                    for msg in messages:
                        expected_id = 2 if msg.get_srcComponent() == 1 else 0
                        if msg.camera_device_id != expected_id:
                            raise NotAchievedException("Mixed camera device ID incorrect: %s" % msg)
                    if message_type == 'CAMERA_INFORMATION':
                        fc_info = next(m for m in messages if m.get_srcComponent() == 1)
                        if fc_info.lens_id != 1:
                            raise NotAchievedException("FC camera lost its legacy instance ID")
                        native_info = next(m for m in messages if m.get_srcComponent() == 100)
                        if native_info.gimbal_device_id != 154:
                            raise NotAchievedException("Native gimbal association was replaced")
                    elif message_type == 'CAMERA_CAPTURE_STATUS':
                        native_status = next(m for m in messages if m.get_srcComponent() == 100)
                        if native_status.video_status != 1 or native_status.image_count != 7:
                            raise NotAchievedException("Native capture status mixed with FC camera")

                camera.mav.camera_information_send(
                    123, b"ArduPilot".ljust(32, b'\0'), b"Native".ljust(32, b'\0'),
                    1, 0, 0, 0, 640, 480, 0, 0, 0, b"", 0)
                self.delay_sim_time(1, reason="cache standalone camera information")
                self.context_clear_collection('CAMERA_INFORMATION')
                self.send_poll_message('CAMERA_INFORMATION')
                self.delay_sim_time(0.5, reason="collect camera mount association")
                native = [m for m in self.context_collection('CAMERA_INFORMATION') if m.get_srcComponent() == 100]
                if len(native) != 1 or native[0].gimbal_device_id != 2:
                    raise NotAchievedException("Missing configured camera mount association: %s" % native)
        finally:
            mavutil.mavfile_global = saved_mavfile_global
            camera.close()
