'''
Copter autotest: BatteryMissing

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BatteryMissing(TestGroup):
    @testcase(bucket="CopterTests1c")
    def BatteryMissing(self):
        ''' Test battery health pre-arm and missing failsafe'''
        self.context_push()

        # Should be good to arm with no changes
        self.wait_ready_to_arm()

        # Make monitor unhealthy, this should result in unhealthy prearm
        self.set_parameters({
            'BATT_VOLT_PIN': -1,
        })

        # Battery should go unhealthy immediately
        self.assert_prearm_failure("Battery 1 unhealthy", other_prearm_failures_fatal=False)

        # Return monitor to health
        self.context_pop()

        self.context_push()
        self.wait_ready_to_arm()

        # take off and then trigger in flight
        self.takeoff(10, mode="LOITER")
        self.set_parameters({
            'BATT_VOLT_PIN': -1,
        })

        # Should trigger missing failsafe
        self.wait_statustext("Battery 1 is missing")

        # Done, reset params and clear failsafe
        self.land_and_disarm()
        self.context_pop()
        self.clear_battery_failsafe()
