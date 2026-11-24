'''
Copter autotest: RateThreadPostFilterGyroLog

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RateThreadPostFilterGyroLog(TestGroup):
    @testcase(bucket="CopterTests1a")
    def RateThreadPostFilterGyroLog(self):
        """Check post-filter gyro logging records every sample while the rate thread decimates."""
        self.set_parameters({
            "FSTRATE_ENABLE": 3,    # fixed divisor: the rate loop takes one gyro sample in FSTRATE_DIV
            "FSTRATE_DIV": 3,
            "INS_RAW_LOG_OPT": 5,   # primary gyro, post-filter only
            "LOG_FILE_RATEMAX": 0,  # GYR is rate limited otherwise
            "LOG_DARM_RATEMAX": 0,
        })
        self.reboot_sitl()

        self.takeoff(5, mode="ALT_HOLD")
        self.land_and_disarm()

        log = self.current_onboard_log_filepath()
        self.assert_log_has_no_dropped_blocks(log)
        dfreader = self.dfreader_for_path(log)
        prev = None
        repeats = 0
        sample_dts = []
        loop_dts = []
        gyro_rates = []
        while True:
            m = dfreader.recv_match(type=['GYR', 'IMU', 'RTDT'])
            if m is None:
                break
            mtype = m.get_type()
            if mtype == 'RTDT':
                loop_dts.append(m.dtAvg)
                continue
            if m.I != 0:
                continue
            if mtype == 'IMU':
                gyro_rates.append(m.GHz)
                continue
            if prev is not None:
                sample_dts.append((m.SampleUS - prev.SampleUS) * 1.0e-6)
                if (m.GyrX, m.GyrY, m.GyrZ) == (prev.GyrX, prev.GyrY, prev.GyrZ):
                    repeats += 1
            prev = m
        steps = len(sample_dts)
        if steps < 1000 or len(loop_dts) == 0 or len(gyro_rates) == 0:
            raise NotAchievedException("Logged %u GYR steps, %u RTDT and %u IMU" % (steps, len(loop_dts), len(gyro_rates)))
        gyro_dt = 1.0 / sorted(gyro_rates)[len(gyro_rates) // 2]
        sample_dt = sorted(sample_dts)[steps // 2]
        loop_dt = sorted(loop_dts)[len(loop_dts) // 2]
        self.progress("gyro %.2f ms, logged %.2f ms, rate loop %.2f ms, %u of %u samples repeat" %
                      (gyro_dt * 1000, sample_dt * 1000, loop_dt * 1000, repeats, steps))
        # a held value can only appear while the rate loop is taking a subset of the samples
        if loop_dt < 2 * gyro_dt:
            raise NotAchievedException("Rate loop at %.2f ms is not decimating" % (loop_dt * 1000))
        # every filtered sample must be logged, not just the ones the rate loop took
        if sample_dt > 1.5 * gyro_dt:
            raise NotAchievedException("Post-filter gyro logged every %.2f ms" % (sample_dt * 1000))
        # and each must be new; a value held for the rate loop repeats in most steps
        if repeats > steps * 0.1:
            raise NotAchievedException("Post-filter gyro log repeats %u of %u samples" % (repeats, steps))
