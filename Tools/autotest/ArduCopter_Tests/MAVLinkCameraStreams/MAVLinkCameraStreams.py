'''
Copter autotest: MAVLinkCameraStreams

AP_FLAKE8_CLEAN
'''

import os
import tty

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAVLinkCameraStreams(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MAVLinkCameraStreams(self):
        '''complete a bounded stream list over a link smaller than the reply'''
        self.set_parameters({"CAM1_TYPE": 6, "CAM2_TYPE": 6,
                             "SERIAL5_PROTOCOL": 2, "SERIAL5_BAUD": 9, "MAV3_OPTIONS": 2})
        master, slave = os.openpty()
        tty.setraw(slave)
        os.set_blocking(master, False)
        saved_mavfile_global = mavutil.mavfile_global

        class PTYMAVLink(mavutil.mavfile):
            def recv(self, n=None):
                try:
                    return os.read(self.fd, n or 4096)
                except BlockingIOError:
                    return b''

            def write(self, buf):
                return os.write(self.fd, buf)

        try:
            # A SITL uart device has a 1 KiB TX buffer. Sixteen full stream
            # records cannot fit, so this exercises scheduler retries.
            self.customise_SITL_commandline(["--serial5=uart:%s:9600" % os.ttyname(slave)])
            slow = PTYMAVLink(master, None, source_system=251)
            mavutil.mavfile_global = saved_mavfile_global
            old_sysid, old_compid = self.mav.mav.srcSystem, self.mav.mav.srcComponent
            try:
                self.mav.mav.srcSystem = self.sysid_thismav()
                for _ in range(15):
                    for instance in range(2):
                        self.mav.mav.srcComponent = mavutil.mavlink.MAV_COMP_ID_CAMERA + instance
                        self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_CAMERA, 0, 0, 0, 0)
                        self.mav.mav.camera_information_send(
                            0, b"ArduPilot".ljust(32, b'\0'), b"Streams".ljust(32, b'\0'), 1,
                            0, 0, 0, 1920, 1080, 0, 0, 0, b"")
                    self.delay_sim_time(1, reason="discover stream test cameras")
            finally:
                self.mav.mav.srcSystem, self.mav.mav.srcComponent = old_sysid, old_compid
            for instance in range(2):
                self.wait_camera_initialised(instance + 1)

            self.progress("Checking video stream cache invalidation")
            self.context_collect('VIDEO_STREAM_INFORMATION')
            self._inject_mt11_video_stream_information(1, 2, "Visible", "rtsp://127.0.0.1:8554/video1")
            self._inject_mt11_video_stream_information(2, 2, "Thermal", "rtsp://127.0.0.1:8554/video2")
            stream_ids = self._poll_mt11_video_stream_ids()
            if stream_ids != [1, 2]:
                raise NotAchievedException("Camera stream cache incomplete: %s" % stream_ids)
            self._inject_mt11_video_stream_information(1, 1, "Visible", "rtsp://127.0.0.1:8554/video1")
            stream_ids = self._poll_mt11_video_stream_ids()
            if stream_ids != [1]:
                raise NotAchievedException("Retained a removed second stream: %s" % stream_ids)
            self._inject_mt11_video_stream_information(0, 0)
            stream_ids = self._poll_mt11_video_stream_ids()
            if stream_ids != []:
                raise NotAchievedException("Retained streams after count became zero: %s" % stream_ids)
            # Deliver the records out of order to ensure the bounded cache is
            # indexed by stream ID rather than packet arrival order.
            self._inject_mt11_video_stream_information(2, 2, "Thermal", "rtsp://127.0.0.1:8554/video2")
            self._inject_mt11_video_stream_information(1, 2, "Visible", "rtsp://127.0.0.1:8554/video1")
            stream_ids = self._poll_mt11_video_stream_ids()
            if stream_ids != [1, 2]:
                raise NotAchievedException("Stream cache is not ID ordered: %s" % stream_ids)

            for instance in range(2):
                for stream_id in range(1, 13):
                    self._inject_mt11_video_stream_information(
                        stream_id, 12, "Camera%u" % instance, "rtsp://" + "x" * 152,
                        compid=mavutil.mavlink.MAV_COMP_ID_CAMERA + instance)
            self.delay_sim_time(0.5, reason="cache camera stream records")
            self.set_parameter("SIM_BAUDLIMIT_EN", 1)
            self.context_collect('VIDEO_STREAM_INFORMATION')
            for _ in range(2):
                while slow.recv_match() is not None:
                    pass
                slow.mav.command_long_send(self.sysid_thismav(), 1, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                           0, mavutil.mavlink.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, 0, 0, 0, 0, 0, 0)
                self.delay_sim_time(0.3, reason="start the slow-link reply")
                self.context_clear_collection('VIDEO_STREAM_INFORMATION')
                self.send_poll_message('VIDEO_STREAM_INFORMATION')
                received = []
                tstart = self.get_sim_time()
                while len(received) < 16 and self.get_sim_time_cached() - tstart < 20:
                    self.mav.recv_match(blocking=True, timeout=0.1)
                    while (msg := slow.recv_match()) is not None:
                        if msg.get_type() == 'VIDEO_STREAM_INFORMATION':
                            if msg.count != 8:
                                raise NotAchievedException("Relayed uncapped stream count: %u" % msg.count)
                            received.append((msg.get_srcComponent(), msg.name, msg.stream_id))
                expected = [(mavutil.mavlink.MAV_COMP_ID_CAMERA + instance, "Camera%u" % instance, stream_id)
                            for instance in range(2) for stream_id in range(1, 9)]
                if received != expected:
                    raise NotAchievedException("Incomplete or repeated stream list: %s" % received)
                primary = [(m.get_srcComponent(), m.name, m.stream_id)
                           for m in self.context_collection('VIDEO_STREAM_INFORMATION')]
                if primary != expected:
                    raise NotAchievedException("Stream reply cursors interfered across links: %s" % primary)
        finally:
            mavutil.mavfile_global = saved_mavfile_global
            os.close(master)
            os.close(slave)

    def _inject_mt11_video_stream_information(
            self, stream_id, count, stream_name="", uri="", compid=mavutil.mavlink.MAV_COMP_ID_CAMERA):
        '''inject VIDEO_STREAM_INFORMATION as the simulated camera'''
        old_src_system = self.mav.mav.srcSystem
        old_src_component = self.mav.mav.srcComponent
        try:
            self.mav.mav.srcSystem = self.sysid_thismav()
            self.mav.mav.srcComponent = compid
            self.mav.mav.video_stream_information_send(
                stream_id,
                count,
                mavutil.mavlink.VIDEO_STREAM_TYPE_RTSP,
                mavutil.mavlink.VIDEO_STREAM_STATUS_FLAGS_RUNNING,
                30,
                1920,
                1080,
                4096000,
                0,
                88,
                stream_name.encode(),
                uri.encode(),
                mavutil.mavlink.VIDEO_STREAM_ENCODING_H264,
            )
        finally:
            self.mav.mav.srcSystem = old_src_system
            self.mav.mav.srcComponent = old_src_component

    def _poll_mt11_video_stream_ids(self):
        '''request cached stream information and return the relayed IDs'''
        self.context_clear_collection('VIDEO_STREAM_INFORMATION')
        self.send_poll_message('VIDEO_STREAM_INFORMATION')
        self.run_cmd_get_ack(
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            10,
        )
        self.delay_sim_time(0.5, reason="collect MT11 stream information")
        # A relayed camera update can race the cached reply. Keep first-seen
        # ordering but do not mistake another copy for another cached stream.
        return list(dict.fromkeys(
            m.stream_id for m in
            self.context_collection('VIDEO_STREAM_INFORMATION')
            if m.get_srcSystem() == self.sysid_thismav() and
            m.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_CAMERA
        ))
