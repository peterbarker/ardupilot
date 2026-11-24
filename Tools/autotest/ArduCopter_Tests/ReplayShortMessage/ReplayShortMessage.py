'''
Copter autotest: ReplayShortMessage

AP_FLAKE8_CLEAN
'''

import subprocess

from pymavlink import mavutil

from pysim import util
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ReplayShortMessage(TestGroup):
    def truncate_log_message(self, src, dst, name, drop_fields, drop_bytes):
        '''copy dataflash log src to dst, removing the last drop_fields
        fields (drop_bytes bytes) from every message called name.
        Returns the original payload length of that message'''
        data = open(src, 'rb').read()
        out = bytearray()
        FMT_TYPE = 128
        lengths = {FMT_TYPE: 89}
        target_type = None
        payload_len = None
        offset = 0
        while offset + 3 <= len(data):
            if data[offset] != 0xA3 or data[offset+1] != 0x95:
                raise ValueError("Bad message header at offset %u in %s" % (offset, src))
            msg_type = data[offset+2]
            if msg_type not in lengths:
                raise ValueError("Message type %u before its format at offset %u in %s" % (msg_type, offset, src))
            length = lengths[msg_type]
            if offset + length > len(data):
                break
            msg = bytearray(data[offset:offset+length])
            if msg_type == FMT_TYPE:
                lengths[msg[3]] = msg[4]
                if bytes(msg[5:9]).rstrip(b'\0').decode() == name:
                    target_type = msg[3]
                    payload_len = msg[4] - 3
                    msg[4] -= drop_bytes
                    fmt = bytes(msg[9:25]).rstrip(b'\0')[:-drop_fields]
                    msg[9:25] = fmt.ljust(16, b'\0')
                    columns = bytes(msg[25:89]).rstrip(b'\0').split(b',')[:-drop_fields]
                    msg[25:89] = b','.join(columns).ljust(64, b'\0')
            elif msg_type == target_type:
                msg = msg[:-drop_bytes]
            out += msg
            offset += length
        if target_type is None:
            raise NotAchievedException("No %s in %s" % (name, src))
        open(dst, 'wb').write(out)
        return payload_len

    @testcase(bucket="CopterTests2b")
    def ReplayShortMessage(self):
        '''test replay of a log whose message is shorter than Replay's structure'''
        # This checks Replay warns and still replays such a log.  The
        # truncated RSLL data is refused by the EKF while GPS is good,
        # so reading past the end of the message would not change the
        # replay; only a sanitizer build of Replay detects that.
        self.set_parameters({
            "LOG_REPLAY": 1,
            "LOG_DISARMED": 1,
            # Replay needs every message
            "LOG_DARM_RATEMAX": 0,
            "LOG_FILE_RATEMAX": 0,
            "LOG_FILE_BUFSIZE": 32767,
        })
        self.reboot_sitl()
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_LOGGING, True, True, True)
        self.wait_ready_to_arm()
        log_filepath = self.current_onboard_log_filepath()

        # position estimates are refused while GPS is good, but they
        # are still recorded in RSLL for Replay:
        loc = self.get_location()
        for accuracy in range(1, 11):
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_EXTERNAL_POSITION_ESTIMATE,
                p1=self.get_sim_time()-0.5,  # transmit time
                p2=0.1,  # processing delay
                p3=accuracy,
                p5=int(loc.lat * 1e7),
                p6=int(loc.lng * 1e7),
                p7=float("NaN"),  # alt
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
                want_result=mavutil.mavlink.MAV_RESULT_FAILED,
            )
        self.delay_sim_time(5, reason="log some more")
        # stop logging so Replay's output is the only new log
        self.set_parameters({
            "LOG_REPLAY": 0,
            "LOG_DISARMED": 0,
        })
        self.reboot_sitl()

        # a log from firmware whose RSLL lacked its last field; kept
        # out of the logs directory so later tests don't see it:
        short_log_filepath = self.buildlogs_path("ReplayShortMessage-short.BIN")
        rsll_len = self.truncate_log_message(log_filepath, short_log_filepath, "RSLL", drop_fields=1, drop_bytes=4)

        self.build_replay()
        old_logs = set(self.log_list())
        replay = subprocess.run(
            ['build/sitl/tool/Replay', short_log_filepath],
            cwd=util.topdir(),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        output = replay.stdout.decode('utf-8', errors='replace')
        if replay.returncode != 0:
            raise NotAchievedException("Replay failed (%d): %s" % (replay.returncode, output[-1000:]))
        warnings = [line for line in output.splitlines() if line.startswith("Warning: RSLL is")]
        self.progress("Replay warnings: %s" % str(warnings))
        expected = "Warning: RSLL is %u bytes in the log but %u bytes in Replay" % (rsll_len - 4, rsll_len)
        if warnings != [expected]:
            raise NotAchievedException("Expected one RSLL length warning")

        # RSLL is not used while GPS is good, so the replay must still
        # match despite the missing field
        check_replay = util.load_local_module("Tools/Replay/check_replay.py")
        new_logs = [x for x in self.log_list() if x not in old_logs]
        if len(new_logs) != 1:
            raise NotAchievedException("Expected one new log from Replay, got %s" % str(new_logs))
        replay_log_filepath = new_logs[0]
        if not check_replay.check_log(replay_log_filepath, self.progress, verbose=True):
            raise NotAchievedException("check_replay (%s) failed" % replay_log_filepath)
