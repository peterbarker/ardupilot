'''
Rover autotest: MAVLinkTimesyncRTT

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAVLinkTimesyncRTT(TestGroup):
    @testcase(bucket="Rover")
    def MAVLinkTimesyncRTT(self):
        """Request cookies must not contribute to RTT, even across ID changes."""
        self.set_parameter("LOG_DISARMED", 1)
        sysid = 0xFFFFFFFF
        self.send_set_parameter_direct("MAV_SYSID", -1)
        self.mav.target_system = sysid
        with self.mavlink_target_system_context():
            try:
                self.wait_heartbeat(timeout=30)
                for change_id in (False, True):
                    request = self.assert_receive_message('TIMESYNC', timeout=30,
                                                          condition='TIMESYNC.tc1 == 0')
                    if change_id:
                        self.send_set_parameter_direct("MAV_SYSID", 100000)
                        sysid = 100000
                        self.mav.target_system = sysid
                        self.wait_heartbeat(timeout=30)
                    self.mav.mav.timesync_send(1, request.ts1)
                    self.delay_sim_time(1, reason="process TIMESYNC response")
                self.delay_sim_time(3, reason="flush TIMESYNC log records")
                log = self.dfreader_for_current_onboard_log()
                count = 0
                while True:
                    record = log.recv_match(type='TSYN')
                    if record is None:
                        break
                    if record.SysID != self.mav.source_system:
                        continue
                    if record.RTT > 5000000:
                        raise NotAchievedException("TIMESYNC RTT includes request cookie: %u" % record.RTT)
                    count += 1
                if count < 2:
                    raise NotAchievedException("Missing TIMESYNC response log records")
            finally:
                self.send_set_parameter_direct("MAV_SYSID", 1)
                self.mav.target_system = 1
                self.wait_heartbeat(timeout=30)
