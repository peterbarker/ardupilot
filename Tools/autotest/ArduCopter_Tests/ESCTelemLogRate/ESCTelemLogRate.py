'''
Copter autotest: ESCTelemLogRate

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ESCTelemLogRate(TestGroup):
    @testcase(bucket="CopterTests1a")
    def ESCTelemLogRate(self):
        '''check ESC_TLM_LOG_RATE sets the rate ESC telemetry is logged at, and 0 stops it'''
        # the harness rate-limits streaming log messages such as ESC; with
        # no limit the default log buffer can drop the blocks being counted
        self.set_parameters({
            "LOG_DARM_RATEMAX": 0,
            "LOG_FILE_RATEMAX": 0,
            "LOG_FILE_BUFSIZE": 32767,
            "SIM_ESC_TELEM": 1,
        })
        self.reboot_sitl()
        # SITL's ESC telemetry starts 6 seconds after boot
        self.assert_receive_message('ESC_TELEMETRY_1_TO_4', timeout=10)
        rates_hz = [10, 50, 0]
        for rate_hz in rates_hz:
            self.set_parameter("ESC_TLM_LOG_RATE", rate_hz)
            self.delay_sim_time(10, "log ESC telemetry at %uHz" % rate_hz)
        # a final change ends the last window
        self.set_parameter("ESC_TLM_LOG_RATE", 100)
        log_filepath = self.current_onboard_log_filepath()

        def rate_windows():
            '''count ESC records per instance between changes of ESC_TLM_LOG_RATE, in log time'''
            dfreader = self.dfreader_for_path(log_filepath)
            windows = []
            while True:
                m = dfreader.recv_match(type=['PARM', 'ESC'])
                if m is None:
                    break
                if m.get_type() == 'ESC':
                    if len(windows):
                        count = windows[-1]["count"]
                        count[m.Instance] = count.get(m.Instance, 0) + 1
                    continue
                if m.Name != 'ESC_TLM_LOG_RATE':
                    continue
                if len(windows):
                    if windows[-1]["rate_hz"] == m.Value:
                        continue
                    windows[-1]["end_us"] = m.TimeUS
                windows.append({"rate_hz": m.Value, "start_us": m.TimeUS, "count": {}})
            return windows[-len(rates_hz)-1:-1]

        # the log is written in order, so once it holds the final change it
        # holds every ESC row before it; the larger buffer takes a few seconds
        tstart = self.get_sim_time()
        while True:
            self.delay_sim_time(2, "the file logger writes out the buffer")
            windows = rate_windows()
            if [w["rate_hz"] for w in windows] == rates_hz:
                break
            if self.get_sim_time_cached() - tstart > 30:
                raise NotAchievedException("Did not find ESC_TLM_LOG_RATE changes %s in log" % str(rates_hz))
        self.assert_log_has_no_dropped_blocks(log_filepath)
        for w in windows:
            duration_s = (w["end_us"] - w["start_us"]) * 1.0e-6
            # the default frame is a quad
            for instance in range(4):
                achieved_hz = w["count"].get(instance, 0) / duration_s
                self.progress("ESC_TLM_LOG_RATE=%u: ESC[%u] logged at %.1fHz over %.1fs" %
                              (w["rate_hz"], instance, achieved_hz, duration_s))
                if abs(achieved_hz - w["rate_hz"]) > 0.1 * w["rate_hz"]:
                    raise NotAchievedException("ESC[%u] logged at %.1fHz, want %uHz" %
                                               (instance, achieved_hz, w["rate_hz"]))
