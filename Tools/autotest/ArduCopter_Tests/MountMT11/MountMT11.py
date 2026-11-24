'''
Copter autotest: MountMT11

AP_FLAKE8_CLEAN
'''

import math
import re
import socket
import time

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MountMT11(TestGroup):
    def _check_mt11_rtsp_requests(self, uri):
        '''reject oversized interleaved frames and accept fragmented requests'''
        address = uri.split("//", 1)[1].split("/", 1)[0]
        host, port = address.rsplit(":", 1)
        for length in (4092, 0xfffb, 0xfffc, 0xfffd, 0xffff):
            with socket.create_connection((host, int(port)), timeout=5) as sock:
                sock.sendall(b'$\x01' + length.to_bytes(2, 'big'))
                if sock.recv(1) != b'':
                    raise NotAchievedException("Oversized RTSP frame was not rejected")
        with socket.create_connection((host, int(port)), timeout=5) as sock:
            # A valid interleaved frame split across reads, followed by an
            # empty frame and two pipelined RTSP requests.
            sock.sendall(b'$\x01\x00')
            time.sleep(0.05)
            sock.sendall(b'\x02ab$\x01\x00\x00')
            request = "OPTIONS %s RTSP/1.0\r\nCSeq: %%u\r\n\r\n" % uri
            sock.sendall(((request % 1) + (request % 2)).encode())
            response = b''
            while response.count(b'\r\n\r\n') < 2:
                data = sock.recv(4096)
                if not data:
                    raise NotAchievedException("RTSP connection closed on valid frames")
                response += data
            if response.count(b'RTSP/1.0 200 OK') != 2:
                raise NotAchievedException("RTSP pipelined responses were lost")

    def _check_mt11_rtsp_stream(self, uri, stream_name):
        '''check an MT11 RTSP stream through to an H264 RTP packet'''
        address = uri.split("//", 1)[1].split("/", 1)[0]
        host, port = address.rsplit(":", 1)
        sock = socket.create_connection((host, int(port)), timeout=5)
        sock.settimeout(5)
        receive_buffer = b''
        cseq = 0

        def receive():
            data = sock.recv(4096)
            if not data:
                raise NotAchievedException(
                    "MT11 %s RTSP connection closed" % stream_name)
            return data

        def request(method, request_uri, headers=None):
            nonlocal cseq, receive_buffer
            cseq += 1
            lines = [
                "%s %s RTSP/1.0" % (method, request_uri),
                "CSeq: %u" % cseq,
                "User-Agent: ArduPilot-AutoTest",
            ]
            if headers is not None:
                lines.extend(headers)
            sock.sendall(("\r\n".join(lines) + "\r\n\r\n").encode())

            while b'\r\n\r\n' not in receive_buffer:
                receive_buffer += receive()
            raw_headers, receive_buffer = receive_buffer.split(b'\r\n\r\n', 1)
            response_headers = raw_headers.decode().split('\r\n')
            if response_headers[0] != 'RTSP/1.0 200 OK':
                raise NotAchievedException(
                    "%s failed for %s: %s" %
                    (method, stream_name, response_headers[0]))
            content_length = 0
            for header in response_headers[1:]:
                if header.lower().startswith('content-length:'):
                    content_length = int(header.split(':', 1)[1])
                    break
            while len(receive_buffer) < content_length:
                receive_buffer += receive()
            body = receive_buffer[:content_length]
            receive_buffer = receive_buffer[content_length:]
            return response_headers, body

        try:
            request('OPTIONS', uri)
            _, sdp = request('DESCRIBE', uri, ['Accept: application/sdp'])
            expected_sdp = [
                ('s=MT11 %s SITL' % stream_name).encode(),
                b'a=rtpmap:96 H264/90000',
                b'a=fmtp:96 packetization-mode=1',
                b'sprop-parameter-sets=',
            ]
            for value in expected_sdp:
                if value not in sdp:
                    raise NotAchievedException(
                        "MT11 %s SDP missing %s" % (stream_name, value))

            headers, _ = request(
                'SETUP', uri + '/trackID=0',
                ['Transport: RTP/AVP/TCP;unicast;interleaved=0-1'])
            session = None
            for header in headers:
                if header.lower().startswith('session:'):
                    session = header.split(':', 1)[1].split(';', 1)[0].strip()
                    break
            if session is None:
                raise NotAchievedException(
                    "MT11 %s SETUP response has no session" % stream_name)
            request('PLAY', uri, ['Session: %s' % session])

            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                marker = receive_buffer.find(b'$')
                if marker > 0:
                    receive_buffer = receive_buffer[marker:]
                if len(receive_buffer) >= 4:
                    packet_length = int.from_bytes(receive_buffer[2:4], 'big')
                    if len(receive_buffer) >= packet_length + 4:
                        channel = receive_buffer[1]
                        packet = receive_buffer[4:packet_length + 4]
                        if (channel == 0 and len(packet) >= 13 and
                                packet[0] >> 6 == 2 and packet[1] & 0x7f == 96):
                            return
                        receive_buffer = receive_buffer[packet_length + 4:]
                        continue
                receive_buffer += receive()
            raise NotAchievedException(
                "Did not receive H264 RTP from MT11 %s stream" % stream_name)
        finally:
            sock.close()

    @testcase(bucket="CopterTests1c")
    def MountMT11(self):
        '''test the MAVLink camera and gimbal protocols using SIM_MT11'''
        self.set_parameters({
            "MNT1_TYPE": 6,         # MAVLink
            "CAM1_TYPE": 6,         # MAVLink Camera v2
            "SERIAL1_PROTOCOL": 2,
            "SERIAL2_PROTOCOL": -1,  # SERIAL0/1/5 map to MAV1/2/3
            "SERIAL5_PROTOCOL": 2,  # MAVLink2
            "MAV3_OPTIONS": 2,     # private link: no forwarded GCS streams
            "MAV3_POSITION": 0,
            "MNT1_ATT_RATE": 0,    # the gimbal must request its own telemetry
        })
        self.customise_SITL_commandline(["--serial5=sim:mt11:"])
        self.wait_camera_initialised(1)

        if self.get_parameter("MNT1_TARG_RATE") != 10:
            raise NotAchievedException("Unexpected MNT1_TARG_RATE default")

        self.progress("Checking MT11 GCS attitude-rate override")
        attitude_message = "AUTOPILOT_STATE_FOR_GIMBAL_DEVICE"
        self.set_message_rate_hz(attitude_message, 4)
        measured_rate = self.measure_message_rate(attitude_message, timeout=5)
        if abs(measured_rate - 4) > 1:
            raise NotAchievedException(
                "MT11 attitude rate override: want=4Hz got=%fHz" %
                measured_rate)
        self.set_message_rate_hz(attitude_message, -1)
        self.drain_mav()
        if self.measure_message_rate(attitude_message, timeout=2) != 0:
            raise NotAchievedException("MT11 attitude messages did not stop")

        self.progress("Checking MT11 camera information")
        info = self.poll_camera_message('CAMERA_INFORMATION', p2=1)
        vendor = bytes(info.vendor_name).split(b'\x00')[0].decode('utf-8')
        model = bytes(info.model_name).split(b'\x00')[0].decode('utf-8')
        if vendor != "ArduPilot" or model != "MT11":
            raise NotAchievedException(
                "Unexpected MT11 identity: vendor=%s model=%s" %
                (vendor, model))
        if info.firmware_version != 1:
            raise NotAchievedException(
                "Unexpected MT11 firmware version: %u" % info.firmware_version)
        if info.flags != 0x1DF:
            raise NotAchievedException(
                "Unexpected MT11 camera capabilities: 0x%x" % info.flags)
        if info.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_CAMERA:
            raise NotAchievedException("Cached camera information lost its source component")
        if info.gimbal_device_id != mavutil.mavlink.MAV_COMP_ID_GIMBAL:
            raise NotAchievedException(
                "Unexpected MT11 gimbal device ID: %u" %
                info.gimbal_device_id)

        self.progress("Checking MT11 video stream information")
        self.context_collect('VIDEO_STREAM_INFORMATION')
        self.context_clear_collection('VIDEO_STREAM_INFORMATION')
        self.send_poll_message('VIDEO_STREAM_INFORMATION')
        self.run_cmd_get_ack(
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            10,
        )
        tstart = self.get_sim_time()
        while {m.stream_id for m in self.context_collection(
                'VIDEO_STREAM_INFORMATION')} != {1, 2}:
            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException("Did not receive both MT11 video streams")
            self.mav.recv_match(blocking=True, timeout=0.1)
        streams = {
            m.stream_id: m
            for m in self.context_collection('VIDEO_STREAM_INFORMATION')
        }
        if set(streams) != {1, 2}:
            raise NotAchievedException(
                "Unexpected MT11 video stream IDs: %s" % sorted(streams))
        expected_streams = {
            1: ("Visible", 1920, 1080, 88,
                mavutil.mavlink.VIDEO_STREAM_STATUS_FLAGS_RUNNING),
            2: ("Thermal", 1280, 720, 24,
                mavutil.mavlink.VIDEO_STREAM_STATUS_FLAGS_RUNNING |
                mavutil.mavlink.VIDEO_STREAM_STATUS_FLAGS_THERMAL),
        }
        for stream_id, expected in expected_streams.items():
            stream = streams[stream_id]
            got = (stream.name, stream.resolution_h, stream.resolution_v,
                   stream.hfov, stream.flags)
            if got != expected:
                raise NotAchievedException(
                    "Unexpected MT11 stream %u: want=%s got=%s" %
                    (stream_id, expected, got))
            if stream.count != 2 or stream.type != mavutil.mavlink.VIDEO_STREAM_TYPE_RTSP:
                raise NotAchievedException(
                    "Unexpected MT11 stream %u count/type" % stream_id)
            if stream.encoding != mavutil.mavlink.VIDEO_STREAM_ENCODING_H264:
                raise NotAchievedException(
                    "Unexpected MT11 stream %u encoding" % stream_id)
            if not re.match(r'^rtsp://127\.0\.0\.1:\d+/video%u$' % stream_id,
                            stream.uri):
                raise NotAchievedException(
                    "Unexpected MT11 stream %u URI: %s" %
                    (stream_id, stream.uri))

        self.progress("Checking MT11 embedded RTSP video streams")
        self._check_mt11_rtsp_requests(streams[1].uri)
        for stream_id in sorted(streams):
            stream = streams[stream_id]
            self._check_mt11_rtsp_stream(stream.uri, stream.name)

        self.progress("Checking MT11 recording status relay")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE,
            p1=1,
        )
        status = self.poll_camera_message('CAMERA_CAPTURE_STATUS')
        if status.video_status != 1:
            raise NotAchievedException("MT11 did not report recording started")
        self.delay_sim_time(1, reason="allow MT11 recording time to advance")
        status = self.poll_camera_message('CAMERA_CAPTURE_STATUS')
        if status.video_status != 1 or status.recording_time_ms < 500:
            raise NotAchievedException("MT11 recording time did not advance")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_VIDEO_STOP_CAPTURE,
            p1=1,
        )
        status = self.poll_camera_message('CAMERA_CAPTURE_STATUS')
        if status.video_status != 0:
            raise NotAchievedException("MT11 did not report recording stopped")

        self.progress("Checking disabled MT11 target transmission")
        self.set_parameter("MNT1_TARG_RATE", 0)
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            p1=-20,
            p2=0,
            p3=float('nan'),
            p4=float('nan'),
        )
        self.delay_sim_time(1, reason="check disabled MT11 target transmission")
        _, pitch, _, _ = self.get_mount_roll_pitch_yaw_deg()
        if abs(pitch - -20) < 5:
            raise NotAchievedException("MT11 target sent while MNT1_TARG_RATE was zero")
        self.set_parameter("MNT1_TARG_RATE", 10)
        self.wait_mount_roll_pitch_yaw_deg(p=-20)

        self.progress("Checking MT11 native global-location targeting")
        here = self.get_location(frame=AltFrame.ABOVE_HOME)
        target = self.offset_location_ne(here, 100, 100)
        target_alt = here.get_alt_m(AltFrame.ABOVE_HOME) + 50
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            x=int(target.lat * 1e7),
            y=int(target.lng * 1e7),
            z=target_alt,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        expected_pitch = math.degrees(math.atan2(50, math.sqrt(2) * 100))
        # Native status retains the gimbal's earth-frame yaw and source ID.
        self.wait_mount_roll_pitch_yaw_deg(p=expected_pitch, y=45)
