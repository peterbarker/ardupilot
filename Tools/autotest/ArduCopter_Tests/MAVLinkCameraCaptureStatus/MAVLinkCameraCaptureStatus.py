'''
Copter autotest: MAVLinkCameraCaptureStatus

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAVLinkCameraCaptureStatus(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MAVLinkCameraCaptureStatus(self):
        '''expire missing camera status and retain locally scheduled interval capture'''
        self.set_parameter("CAM1_TYPE", 6)
        self.reboot_sitl()
        old_sysid, old_compid = self.mav.mav.srcSystem, self.mav.mav.srcComponent

        def camera_send(send):
            try:
                self.mav.mav.srcSystem = self.sysid_thismav()
                self.mav.mav.srcComponent = mavutil.mavlink.MAV_COMP_ID_CAMERA
                send()
            finally:
                self.mav.mav.srcSystem, self.mav.mav.srcComponent = old_sysid, old_compid

        capture_flags = mavutil.mavlink.CAMERA_CAP_FLAGS_CAPTURE_IMAGE | mavutil.mavlink.CAMERA_CAP_FLAGS_CAPTURE_VIDEO

        def camera_information(flags):
            camera_send(lambda: self.mav.mav.camera_information_send(
                0, b"ArduPilot".ljust(32, b'\0'), b"Test".ljust(32, b'\0'), 1,
                0, 0, 0, 1920, 1080, 0, flags, 0, b""))

        for _ in range(15):
            camera_send(lambda: self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_CAMERA, 0, 0, 0, 0))
            camera_information(capture_flags)
            self.delay_sim_time(1, reason="discover test camera")
        self.poll_camera_message('CAMERA_INFORMATION')

        def status(image_status=0, video_status=1):
            camera_send(lambda: self.mav.mav.camera_capture_status_send(123, image_status, video_status, 0, 456, 789, 10))
            self.delay_sim_time(0.1, reason="receive remote camera status")

        status()
        msg = self.poll_camera_message('CAMERA_CAPTURE_STATUS')
        if msg.video_status != 1 or msg.recording_time_ms != 456 or msg.time_boot_ms != 123:
            raise NotAchievedException("Incorrect relayed camera status")
        self.run_cmd(mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE, p1=1, p2=1, p3=0)
        status(image_status=1)
        msg = self.poll_camera_message('CAMERA_CAPTURE_STATUS')
        if msg.image_status != 3 or abs(msg.image_interval - 1) > 0.01:
            raise NotAchievedException("Lost local interval or remote capture-in-progress state")
        self.run_cmd(mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE, p1=1)
        status()
        if self.poll_camera_message('CAMERA_CAPTURE_STATUS').image_status != 0:
            raise NotAchievedException("Interval capture did not stop")

        self.delay_sim_time(4, reason="expire remote camera status")
        self.context_collect('CAMERA_CAPTURE_STATUS')
        self.send_poll_message('CAMERA_CAPTURE_STATUS')
        self.delay_sim_time(1, reason="check expired status is not relayed")
        if self.context_collection('CAMERA_CAPTURE_STATUS'):
            raise NotAchievedException("Expired camera status was relayed")
        status(video_status=0)
        if self.poll_camera_message('CAMERA_CAPTURE_STATUS').video_status != 0:
            raise NotAchievedException("Camera status did not recover after expiry")

        self.context_collect('COMMAND_LONG')

        def count_status_requests(duration):
            self.drain_mav()
            self.context_clear_collection('COMMAND_LONG')
            self.delay_sim_time(duration, reason="measure camera status polling")
            return len([m for m in self.context_collection('COMMAND_LONG')
                        if m.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE and
                        m.param1 == mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS and
                        m.target_component == mavutil.mavlink.MAV_COMP_ID_CAMERA])

        self.delay_sim_time(4, reason="allow three unanswered status requests")
        count = count_status_requests(22)
        if not 2 <= count <= 3:
            raise NotAchievedException("Missing camera status did not back off: %u requests" % count)
        camera_information(0)
        self.delay_sim_time(1, reason="remove camera capture capabilities")
        if count_status_requests(12) != 0:
            raise NotAchievedException("Polled capture status without capture capabilities")
        camera_information(capture_flags)
        status()
        if not 2 <= count_status_requests(3.2) <= 3:
            raise NotAchievedException("Camera status polling did not recover to 1 Hz")
