'''
Copter autotest: DefaultIntervalsFromFiles

AP_FLAKE8_CLEAN
'''

import os

from pymavlink import mavutil

from pysim import util

from test_discovery import TestGroup
from test_discovery import testcase


class DefaultIntervalsFromFiles(TestGroup):
    @testcase(bucket="CopterTests2a")
    def DefaultIntervalsFromFiles(self):
        '''Test setting default mavlink message intervals from files'''
        ex = None
        intervals_filepath = util.reltopdir("message-intervals-chan0.txt")
        self.progress("Using filepath (%s)" % intervals_filepath)
        try:
            with open(intervals_filepath, "w") as f:
                f.write("""30 50
28 100
29 200
""")
                f.close()

            # other tests may have explicitly set rates, so wipe parameters:
            def custom_stream_rate_setter():
                for stream in mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS:
                    self.set_streamrate(5, stream=stream)

            self.customise_SITL_commandline(
                [],
                wipe=True,
                set_streamrate_callback=custom_stream_rate_setter,
            )

            self.assert_message_rate_hz("ATTITUDE", 20)
            self.assert_message_rate_hz("SCALED_PRESSURE", 5)

        except Exception as e:  # noqa: BLE001
            self.print_exception_caught(e)
            ex = e

        os.unlink(intervals_filepath)

        self.reboot_sitl()

        if ex is not None:
            raise ex
