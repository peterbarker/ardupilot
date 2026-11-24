'''
Copter autotest: MT11MAVFTP

AP_FLAKE8_CLEAN
'''

import logging
import os
import re
import tempfile
import xml.etree.ElementTree as ET

from pymavlink import mavftp_op
from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP
from pymavlink.mavftp import FtpError
from pymavlink.mavftp_op import FTP_OP

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MT11MAVFTP(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MT11MAVFTP(self):
        '''list and download the simulated camera definition through a unicast link'''
        self.set_parameters({
            "MNT1_TYPE": 6,
            "CAM1_TYPE": 6,
            "SERIAL1_PROTOCOL": 2,
            "SERIAL2_PROTOCOL": -1,  # SERIAL0/1/5 map to MAV1/2/3
            "SERIAL5_PROTOCOL": 2,
            "MAV3_OPTIONS": 16,  # unicast, not the legacy private-link option
            "MNT1_ATT_RATE": 0,
        })
        self.customise_SITL_commandline(["--serial5=sim:mt11:"])
        self.wait_camera_initialised(1)
        info = self.poll_camera_message('CAMERA_INFORMATION', p2=1)
        if info.cam_definition_version != 1:
            raise NotAchievedException("Missing MT11 camera definition version")
        # The URI selects the file server, independently of the camera's
        # source identity used for camera controls and extended parameters.
        uri = re.fullmatch(r'mftp://\[;comp=(\d+)\](/camera.xml)', info.cam_definition_uri)
        if uri is None or int(uri[1]) != mavutil.mavlink.MAV_COMP_ID_CAMERA:
            raise NotAchievedException("Unexpected camera definition URI: %s" % info.cam_definition_uri)
        target_system = info.get_srcSystem()
        target_component = int(uri[1])
        path = uri[2]
        replies = []

        def check_ftp_identity(mav, msg):
            if msg.get_type() != 'FILE_TRANSFER_PROTOCOL':
                return
            if (msg.get_srcSystem() != target_system or msg.get_srcComponent() != target_component or
                    msg.target_system != mav.source_system or msg.target_component != mav.source_component):
                raise NotAchievedException("FTP reply lost its camera/GCS routing identity: %s" % msg)
            replies.append(msg)

        self.install_message_hook_context(check_ftp_identity)
        # Scope pymavlink's logging handler to this test's output stream.
        # basicConfig would otherwise retain the closed per-test TeeBoth.
        log_handler = logging.StreamHandler()
        logging.getLogger().addHandler(log_handler)
        try:
            client = MAVFTP(self.mav, target_system, target_component)
            result = client.cmd_list(['/'])
            if result.error_code != FtpError.Success or len(client.list_result) != 1:
                raise NotAchievedException("MT11 FTP directory listing failed: %s" % result.error_code)
            entry = client.list_result[0]
            if entry.name != 'camera.xml' or entry.is_dir or entry.size_b <= 239:
                raise NotAchievedException("Unexpected MT11 directory entry: %s" % entry)

            with tempfile.TemporaryDirectory(prefix='mt11-ftp-') as directory:
                filename = os.path.join(directory, 'camera.xml')
                client.temp_filename = os.path.join(directory, 'partial.xml')
                client.ftp_settings.burst_read_size = 239
                client.cmd_get([path, filename])
                result = client.process_ftp_reply('OpenFileRO')
                if result.error_code != FtpError.Success or not os.path.isfile(filename):
                    raise NotAchievedException("MT11 FTP download failed: %s" % result.error_code)
                with open(filename, 'rb') as downloaded:
                    data = downloaded.read()
        finally:
            logging.getLogger().removeHandler(log_handler)
            log_handler.setStream(None)
            log_handler.close()
        if len(data) != entry.size_b:
            raise NotAchievedException("MT11 FTP listing and download sizes differ")
        root = ET.fromstring(data)
        definition = root.find('definition')
        if (root.tag != 'mavlinkcamera' or definition is None or
                definition.get('version') != str(info.cam_definition_version) or
                definition.findtext('model') != 'MT11' or definition.findtext('vendor') != 'ArduPilot'):
            raise NotAchievedException("Invalid MT11 camera definition: %s" % data)
        burst_replies = [m for m in replies if m.payload[3] == mavftp_op.OP_Ack and
                         m.payload[5] == mavftp_op.OP_BurstReadFile]
        if len(burst_replies) < 2:
            raise NotAchievedException("Camera download did not exercise multiple FTP packets")

        def request(seq, opcode, size=0, offset=0, payload=None, session=7, error=None):
            op = FTP_OP(seq, session, opcode, size, 0, 0, offset, payload)
            packed = op.pack()
            packed.extend(bytearray(251 - len(packed)))
            self.mav.mav.file_transfer_protocol_send(0, target_system, target_component, packed)
            reply = self.ftp_recv()
            expected_opcode = mavftp_op.OP_Ack if error is None else mavftp_op.OP_Nack
            if (reply is None or reply.seq != (seq + 1) % 65536 or reply.req_opcode != opcode or
                    reply.opcode != expected_opcode or reply.session != session or reply.offset != offset):
                raise NotAchievedException("Incorrect MT11 FTP response: %s" % reply)
            if error is not None and reply.payload != bytearray([error]):
                raise NotAchievedException("Incorrect MT11 FTP error: %s" % reply)
            return reply

        request(1000, mavftp_op.OP_ResetSessions)
        request(1001, mavftp_op.OP_ReadFile, size=80, error=FtpError.InvalidSession)
        request(1002, mavftp_op.OP_OpenFileRO, size=7, payload=b'missing', error=FtpError.FileNotFound)
        request(1003, mavftp_op.OP_ListDirectory, size=1, payload=b'/', offset=1, error=FtpError.EndOfFile)
        request(1004, mavftp_op.OP_ListDirectory, size=240, error=FtpError.InvalidDataSize)
        request(1005, 255, error=FtpError.UnknownCommand)
        opened = request(1006, mavftp_op.OP_OpenFileRO, size=len(path), payload=path.encode('ascii'))
        repeated = request(1006, mavftp_op.OP_OpenFileRO, size=len(path), payload=path.encode('ascii'))
        if opened.pack() != repeated.pack() or int.from_bytes(opened.payload, 'little') != len(data):
            raise NotAchievedException("MT11 open retry changed the file/session")
        request(1007, mavftp_op.OP_ReadFile, size=80, session=8, error=FtpError.InvalidSession)
        request(1008, mavftp_op.OP_OpenFileRO, size=len(path), payload=path.encode('ascii'),
                session=8, error=FtpError.NoSessionsAvailable)
        ordinary_data = bytearray()
        seq = 1009
        while len(ordinary_data) < len(data):
            reply = request(seq, mavftp_op.OP_ReadFile, size=80, offset=len(ordinary_data))
            if not reply.payload:
                raise NotAchievedException("Empty MT11 read before EOF")
            ordinary_data.extend(reply.payload)
            seq += 1
        if ordinary_data != data:
            raise NotAchievedException("MT11 ordinary and burst downloads differ")
        request(seq, mavftp_op.OP_ReadFile, size=80, offset=len(data), error=FtpError.EndOfFile)
        request(seq + 1, mavftp_op.OP_BurstReadFile, offset=0xffffffff, error=FtpError.EndOfFile)
        request(seq + 2, mavftp_op.OP_TerminateSession)
        request(seq + 2, mavftp_op.OP_TerminateSession)
        request(seq + 3, mavftp_op.OP_ReadFile, size=80, error=FtpError.InvalidSession)
