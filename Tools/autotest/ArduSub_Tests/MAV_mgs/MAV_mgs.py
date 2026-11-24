'''
Sub autotest: MAV_mgs

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_mgs(TestGroup):
    @testcase(bucket="Sub")
    def MAV_mgs(self):
        '''test individual GCS backends timestamps'''
        self.reboot_sitl()
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)
        self.delay_sim_time(10, reason='add delay on connecting "telemetry')

        self.progress("Connecting to telemetry port")
        mav2 = mavutil.mavlink_connection(
            self.sitl_serial_endpoint(2),
            robust_parsing=True,
            source_system=self.mav.source_system,
            source_component=self.mav.source_component,
        )
        tstart = self.get_sim_time()
        while True:
            tnow = self.get_sim_time()
            self.drain_mav()
            if tnow - tstart > 20:
                break
            if tnow - tstart > 1:
                mav2.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
        self.delay_sim_time(20, reason="allow for checking not receiving hb any more on chan=2")
        dfreader = self.dfreader_for_current_onboard_log()

        chan0_count = 0
        chan0_last_timestamp_us = 0
        chan0_last_mgs = 0
        chan2_count = 0
        chan2_last_timestamp_us = 0
        chan2_last_mgs = 0
        att_ts_us = 0
        gcs_sysid_set_ts_us = None
        while True:
            m = dfreader.recv_match(type=['MAV', 'ATT', 'PARM'])
            if m is None:
                raise NotAchievedException("Did not find everything wanted in log")
            if chan2_count > 10:
                self.progress("Received 10 heartbeats on chan==2")
                break
            if m.get_type() == 'PARM':
                # the moment MAV_GCS_SYSID took effect; heartbeats from
                # us only count towards mgs from here on.  Measuring
                # chan 0's latching relative to this rather than to
                # boot keeps the check independent of how much
                # simulated time the reboot-detection polling burnt,
                # which at high speedup can be many seconds.
                if m.Name == 'MAV_GCS_SYSID' and int(m.Value) == self.mav.source_system:
                    gcs_sysid_set_ts_us = m.TimeUS
                continue
            if m.get_type() == 'ATT':
                att_ts_us = m.TimeUS
                continue
            if m.mgs == 0:
                # no heartbeat received yet
                continue
            if m.chan == 0:
                if chan0_count == 0:
                    if gcs_sysid_set_ts_us is None:
                        raise NotAchievedException(f"mgs nonzero on chan=0 before MAV_GCS_SYSID was set {att_ts_us=}")
                    if att_ts_us - gcs_sysid_set_ts_us > 5000000:
                        raise NotAchievedException(f"Late arrival on chan=0 {att_ts_us=} {gcs_sysid_set_ts_us=}")
                chan0_count += 1
                if chan0_count > 3:
                    if att_ts_us - chan0_last_timestamp_us > 2000000:
                        raise NotAchievedException(f"Unexpected interval on chan=0 {att_ts_us=} {chan0_last_timestamp_us=}")
                    if m.mgs - chan0_last_mgs > 2000:
                        raise NotAchievedException(f"Unexpected interval on chan==0 mgs {m.mgs=} {chan0_last_mgs=}")
                chan0_last_mgs = m.mgs
                chan0_last_timestamp_us = att_ts_us
            elif m.chan == 2:
                if chan2_count == 0:
                    if att_ts_us < 10000000:
                        raise NotAchievedException(f"Early heartbeat on chan==2 {att_ts_us=}")
                chan2_count += 1
                if chan2_count > 1:
                    if att_ts_us - chan2_last_timestamp_us > 2000000:
                        raise NotAchievedException("Unexpected interval on chan=0")
                    if m.mgs - chan2_last_mgs > 2000:
                        raise NotAchievedException(f"Unexpected interval on chan==0 mgs {m.mgs=} {chan2_last_mgs=}")
                chan2_last_mgs = m.mgs
                chan2_last_timestamp_us = att_ts_us

        self.progress("Waiting for heartbeats to stop on chan==2")
        chan0_last_timestamp_us = 0
        chan2_last_timestamp_us = 0
        while True:
            m = dfreader.recv_match(type=['MAV', 'ATT'])
            if m is None:
                raise NotAchievedException("heartbeats did not stop on chan==2")

            if m.get_type() == 'ATT':
                att_ts_us = m.TimeUS
                continue
            if m.chan == 0:
                chan0_last_timestamp_us = att_ts_us
                if chan0_last_timestamp_us - chan2_last_timestamp_us > 5000000:
                    self.progress("chan==2 heartbeats have stopped")
                    break
            elif m.chan == 2:
                chan2_last_timestamp_us = att_ts_us
