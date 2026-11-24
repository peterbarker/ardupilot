'''
Rover autotest: MAVLinkStatustextWideSource

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAVLinkStatustextWideSource(TestGroup):
    @testcase(bucket="Rover")
    def MAVLinkStatustextWideSource(self):
        """Preserve the source prefix and all payload bytes in MSG logs."""
        self.set_parameter("LOG_DISARMED", 1)
        self.delay_sim_time(2, reason="start logging")
        original_source = self.mav.mav.srcSystem
        original_component = self.mav.mav.srcComponent
        expected = []
        try:
            for source, component in ((42, 190), (100000, 190), (0xFFFFFFFF, 255)):
                self.mav.mav.srcSystem = source
                self.mav.mav.srcComponent = component
                first = "wide-source-test:" + "x" * 33
                last = "end-of-message"
                self.mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,
                                             first.encode(), id=123, chunk_seq=0)
                self.mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,
                                             last.encode(), id=123, chunk_seq=1)
                expected.append("SRC=%u/%u:%s%s" % (source, component, first, last))
                self.delay_sim_time(1, reason="process source's STATUSTEXT chunks")
            # A separate wide-source prefix must not wrap the final payload's
            # log sequence back to zero when MAVLink uses all 256 chunks.
            chunks = []
            for seq in range(256):
                text = ("chunk-%03u:" % seq).ljust(50, "x") if seq < 255 else "last-chunk"
                chunks.append(text)
                self.mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,
                                             text.encode(), id=124, chunk_seq=seq)
                if seq % 16 == 15:
                    # Drain each batch without delay_sim_time's own STATUSTEXT
                    # progress message interrupting the incoming chunk stream.
                    deadline = self.get_sim_time() + 0.2
                    while self.get_sim_time(drain_mav=False) < deadline:
                        pass
            expected.append("SRC=4294967295/255:" + "".join(chunks))
        finally:
            self.mav.mav.srcSystem = original_source
            self.mav.mav.srcComponent = original_component
        self.delay_sim_time(3, reason="flush STATUSTEXT log records")
        log = self.dfreader_for_current_onboard_log()
        messages = {}
        while True:
            record = log.recv_match(type='MSG')
            if record is None:
                break
            if record.Seq == 0:
                messages[record.ID] = record.Message
            elif record.ID in messages:
                messages[record.ID] += record.Message
        for message in expected:
            if message not in messages.values():
                raise NotAchievedException("Missing logged message (%u bytes): %.80s..." % (len(message), message))
