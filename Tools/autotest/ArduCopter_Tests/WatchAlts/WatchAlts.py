'''
Copter autotest: WatchAlts

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class WatchAlts(TestGroup):
    @testcase(bucket="CopterTests2c")
    def WatchAlts(self):
        '''Ensure we can monitor different altitudes'''
        self.takeoff(30, mode='GUIDED')
        self.delay_sim_time(5, reason='let altitude settle')

        self.progress("Testing absolute altitudes")
        absolute_alt = self.get_altitude(altitude_source='SIM_STATE.alt')
        self.progress("absolute_alt=%f" % absolute_alt)
        epsilon = 4  # SIM_STATE and vehicle state can be off by a bit...
        for source in ['GLOBAL_POSITION_INT.alt', 'SIM_STATE.alt', 'GPS_RAW_INT.alt']:
            self.wait_altitude(
                absolute_alt-epsilon,
                absolute_alt+epsilon,
                relative=True,
                altitude_source=source,
                minimum_duration=5,
                timeout=6,
            )

        self.progress("Testing absolute altitudes")
        relative_alt = self.get_altitude(relative=True)
        for source in ['GLOBAL_POSITION_INT.relative_alt']:
            self.wait_altitude(
                relative_alt-epsilon,
                relative_alt+epsilon,
                relative=True,
                altitude_source=source,
                minimum_duration=5,
                timeout=6,
            )

        self.do_RTL()
