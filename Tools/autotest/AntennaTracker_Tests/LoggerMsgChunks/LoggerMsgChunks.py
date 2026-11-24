'''
Tracker autotest: LoggerMsgChunks

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class LoggerMsgChunks(TestGroup):
    @testcase(bucket="Tracker")
    def LoggerMsgChunks(self):
        '''create MSG dataflash entries for very long messages'''
        self.assert_parameter_value('LOG_DISARMED', 1)
        short_message_text = "This is a short message"
        long_message_text = "This is undubitably a ridiculously verbose and needlessly wordy missive, unavoidably designed to exceed disappointingly minuscule log buffers"  # noqa:E501
        self.send_statustext(short_message_text)
        self.send_statustext(long_message_text)

        self.delay_sim_time(10, reason="statustext to be logged")
        dfreader = self.dfreader_for_current_onboard_log()
        self.reboot_sitl()

        phase = "short"
        seq = 0
        received_long_message_text = ""
        while True:
            m = dfreader.recv_match(type='MSG')
            if m is None:
                break
            self.progress(f"{m.Message=}")
            msg = m.Message.removeprefix("SRC=250/250:")
            if phase == "short":
                if msg != short_message_text:
                    continue
                self.progress("Received short message")
                phase = "long"
            elif phase == "long":
                if m.Seq != seq:
                    received_long_message_text = ""
                    seq = 0
                    if m.Seq != 0:
                        raise NotAchievedException("Weird")
                else:
                    seq += 1
                received_long_message_text += msg
                self.progress(f"Text: {received_long_message_text}")
                if received_long_message_text == long_message_text:
                    phase = "done"
                    break

        if phase != "done":
            raise NotAchievedException(f"Did not get message text; {phase=}")  # noqa:E501
