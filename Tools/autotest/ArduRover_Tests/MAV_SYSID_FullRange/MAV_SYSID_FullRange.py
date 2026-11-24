'''
Rover autotest: MAV_SYSID_FullRange

AP_FLAKE8_CLEAN
'''

import struct

from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_SYSID_FullRange(TestGroup):
    @testcase(bucket="Rover")
    def MAV_SYSID_FullRange(self):
        """Preserve unsigned system IDs through MAVFTP, reboot and consumers."""
        self.set_parameters({"FOLL_ENABLE": 1, "SCR_ENABLE": 1, "RC_OVERRIDE_TIME": -1})
        self.install_script_content_context("sysid32-follow.lua", """
local function update()
    gcs:send_named_float("FOLL_HAVE", follow:have_target() and 1 or 0)
    return update, 100
end
return update()
""")
        self.reboot_sitl()
        self.wait_ready_to_arm()
        current_sysid = self.sysid_thismav()

        def read_params():
            data, _ = self.ftp_burst_read("@PARAM/param.pck")
            decoded = MAVFTP.ftp_param_decode(bytes(data))
            if decoded is None:
                raise NotAchievedException("Could not decode MAVFTP parameters")
            return {name.decode(): int(value) & 0xFFFFFFFF for name, value, _ in decoded.params
                    if name.decode() in names}

        def write_params(values):
            nonlocal current_sysid
            records = bytearray()
            for name, value in values.items():
                encoded = name.encode()
                # AP_PARAM_INT32 carries the exact unsigned bit pattern.
                records.extend(bytes([3, (len(encoded) - 1) << 4]) + encoded + struct.pack('<I', value))
            self.ftp_write_file("@PARAM/param.pck", struct.pack('<HHH', 0x671b, len(values), len(records) + 6) + records)
            if 'MAV_SYSID' in values:
                current_sysid = values['MAV_SYSID']
                self.mav.target_system = current_sysid
                self.wait_heartbeat(timeout=30)

        names = ('MAV_SYSID', 'MAV_GCS_SYSID', 'MAV_GCS_SYSID_HI', 'FOLL_SYSID')
        initial = read_params()
        saved = {name: initial[name] for name in names}
        with self.mavlink_target_system_context():
            peer = None
            try:
                for sysid in (0x01000001, 0x7FFFFFFF, 0x80000000, 0xFFFFFFFE, 0xFFFFFFFF):
                    self.progress("Checking exact parameter ID %u" % sysid)
                    expected = dict.fromkeys(names, sysid)
                    write_params(expected)
                    actual = read_params()
                    for name, value in expected.items():
                        if actual[name] != value:
                            raise NotAchievedException("MAVFTP truncated %s: %u != %u" % (name, actual[name], value))
                    self.reboot_sitl()
                    actual = read_params()
                    if any(actual[name] != value for name, value in expected.items()):
                        raise NotAchievedException("Unsigned system IDs changed across reboot: expected %s, got %s" %
                                                   (expected, actual))
                    heartbeat = self.wait_heartbeat()
                    if heartbeat.get_srcSystem() != sysid:
                        raise NotAchievedException("Vehicle source ID was truncated")

                self.wait_ready_to_arm()
                peer = mavutil.mavlink_connection(
                    "tcp:localhost:%u" % self.adjust_ardupilot_port(5763),
                    source_system=0xFFFFFFFE, source_component=7)
                # A follow target in the upper half must not be treated as a
                # negative/unset parameter, nor confused with its low-byte alias.
                write_params({'FOLL_SYSID': 0xFFFFFFFE})
                position = self.assert_receive_message('GLOBAL_POSITION_INT')
                for source, expected in ((254, 0), (0xFFFFFFFE, 1)):
                    peer.mav.srcSystem = source
                    tstart = self.get_sim_time()
                    while self.get_sim_time_cached() - tstart < 3:
                        peer.mav.global_position_int_send(
                            0, position.lat, position.lon, position.alt, position.relative_alt, 0, 0, 0, 0)
                        self.delay_sim_time(0.1, reason="update follow estimate")
                    message = self.mav.recv_match(
                        type='NAMED_VALUE_FLOAT', condition='NAMED_VALUE_FLOAT.name == "FOLL_HAVE"',
                        blocking=True, timeout=5)
                    if message is None or int(message.value) != expected:
                        raise NotAchievedException("Follow mishandled unsigned source %u" % source)

                self.set_rc(1, 1500)
                # Exercise a range crossing INT32_MAX and the disabled-upper-bound
                # sentinel when the lower bound itself has bit 31 set.
                for low, high, cases in (
                        (0x7FFFFFFF, 0xFFFFFFFF, ((0x80000000, True), (0xFFFFFFFF, True), (250, False))),
                        (0x80000000, 0, ((0x80000000, True), (0x80000001, False)))):
                    write_params({'MAV_GCS_SYSID': low, 'MAV_GCS_SYSID_HI': high})
                    for source, accepted in cases:
                        peer.mav.srcSystem = source
                        peer.mav.rc_channels_override_send(current_sysid, 1, 1600, *([65535] * 7))
                        self.delay_sim_time(0.5, reason="process RC override")
                        self.assert_rc_channel_value(1, 1600 if accepted else 1500)
                        if accepted:
                            peer.mav.rc_channels_override_send(current_sysid, 1, 0, *([65535] * 7))
                            self.wait_rc_channel_value(1, 1500)
            finally:
                if peer is not None:
                    peer.close()
                write_params(saved)
