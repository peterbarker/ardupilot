'''
Plane autotest: ClimbThrottleSaturation

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ClimbThrottleSaturation(TestGroup):
    @testcase(bucket="PlaneTests1b", disabled="requires https://github.com/ArduPilot/ardupilot/pull/27106 to pass")
    def ClimbThrottleSaturation(self):
        '''check what happens when throttle is saturated in GUIDED'''
        self.set_parameters({
            "TECS_CLMB_MAX": 30,
            "TKOFF_ALT": 1000,
        })

        self.change_mode("TAKEOFF")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_message_field_values('VFR_HUD', {
            "throttle": 100,
        }, minimum_duration=30, timeout=90)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
