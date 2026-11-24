'''
Copter autotest: ModeFollow_TimestampJitter

AP_FLAKE8_CLEAN
'''

import copy

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ModeFollow_TimestampJitter(TestGroup):
    def send_jittered_follow_samples(self, msg_type):
        '''send a target moving north at 20 m/s at 2 Hz, each sample keeping
        its sample timestamp but delivered 20 to 425 ms late, followed by a
        30 m jump east.  Returns the sim time window the samples were sent in
        and the number of steady samples'''
        vel_n = 20  # m/s
        sample_interval = 0.5
        delays = [0.020, 0.425, 0.220, 0.060, 0.380, 0.140, 0.300, 0.020]
        num_steady = 24
        jump_e = 30

        start_loc = self.get_location()
        self.location_offset_ne(start_loc, 50, 0)
        alt_abs_m = start_loc.get_alt_m(AltFrame.ABSOLUTE)
        origin = self.poll_message('GPS_GLOBAL_ORIGIN')

        self.assert_receive_message('SYSTEM_TIME')
        t0 = self.get_sim_time_cached() + 1
        samples = []
        for k in range(num_steady + 2):
            sample_time = t0 + k * sample_interval
            loc = copy.copy(start_loc)
            self.location_offset_ne(loc, vel_n * (sample_time - t0), jump_e if k >= num_steady else 0)
            samples.append((sample_time + delays[k % len(delays)], sample_time, loc))

        for (send_time, sample_time, loc) in samples:
            while self.get_sim_time_cached() < send_time:
                self.assert_receive_message('SYSTEM_TIME')
            if msg_type == 'GLOBAL_POSITION_INT':
                self.mav.mav.global_position_int_send(
                    int(sample_time * 1000),  # time_boot_ms
                    int(loc.lat * 1e7),
                    int(loc.lng * 1e7),
                    int(alt_abs_m * 1000),  # alt in mm
                    int(alt_abs_m * 1000 - origin.altitude),  # relative alt in mm
                    int(vel_n * 100),  # vx in cm/s
                    0,
                    0,
                    0  # hdg
                )
            else:
                self.mav.mav.follow_target_send(
                    int(sample_time * 1000),  # timestamp in ms
                    1 << 0 | 1 << 1,  # pos, vel
                    int(loc.lat * 1e7),
                    int(loc.lng * 1e7),
                    alt_abs_m,  # alt in m
                    [vel_n, 0, 0],  # velocity m/s
                    [0, 0, 0],  # acceleration m/s/s
                    [1, 0, 0, 0],  # attitude quaternion
                    [0, 0, 0],  # rates
                    [0, 0, 0],  # covariances
                    0  # custom state
                )
        return (samples[0][0] - 0.25, samples[-1][0] + 0.25, num_steady)

    @testcase(bucket="CopterTests1d")
    def ModeFollow_TimestampJitter(self):
        '''check target timestamp jitter does not reset the follow estimate'''
        self.set_parameters({
            "FOLL_ENABLE": 1,
            "FOLL_SYSID": self.mav.source_system,
            "FOLL_TIMEOUT": 2,
            "FOLL_ACCEL_NE": 5,
            "FOLL_DIST_MAX": 0,  # the target travels well beyond the default 100 m
        })
        self.takeoff(10, mode="LOITER")
        self.context_push()
        self.context_set_speedup(1)
        self.set_message_rate_hz('SYSTEM_TIME', 50)

        windows = {}
        for msg_type in 'GLOBAL_POSITION_INT', 'FOLLOW_TARGET':
            self.start_subtest("Jittered %s" % msg_type)
            windows[msg_type] = self.send_jittered_follow_samples(msg_type)

        self.context_pop()
        self.do_RTL()

        # FOLL is written on each target update straight after the reset check,
        # and its estimated location is zero when the estimate has just been reset
        dfreader = self.dfreader_for_current_onboard_log()
        rows = []
        while True:
            m = dfreader.recv_match(type='FOLL')
            if m is None:
                break
            rows.append(m)

        for (msg_type, (tstart, tend, num_steady)) in windows.items():
            valid = [m.LatE != 0 for m in rows if tstart * 1e6 <= m.TimeUS <= tend * 1e6]
            self.progress("%s FOLL estimate valid: %s" % (msg_type, str(valid)))
            if len(valid) != num_steady + 2:
                raise NotAchievedException("%s: expected %u FOLL rows, got %u" % (msg_type, num_steady + 2, len(valid)))
            # the first sample builds the estimate, so its row is skipped
            resets = valid[1:num_steady].count(False)
            if resets != 0:
                raise NotAchievedException("%s: estimate reset on %u of %u steady samples" %
                                           (msg_type, resets, num_steady - 1))
            if valid[num_steady]:
                raise NotAchievedException("%s: estimate not reset by the 30 m jump" % msg_type)
            if not valid[num_steady + 1]:
                raise NotAchievedException("%s: estimate not valid on the update after the jump" % msg_type)
