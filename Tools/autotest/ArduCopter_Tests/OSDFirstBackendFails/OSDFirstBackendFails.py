'''
Copter autotest: OSDFirstBackendFails

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class OSDFirstBackendFails(TestGroup):
    @testcase(bucket="CopterTests2b")
    def OSDFirstBackendFails(self):
        '''check the second OSD runs, and arming is allowed, when the first one does not start'''
        self.context_collect('STATUSTEXT')
        # MSP DisplayPort with no serial port set up for it fails to start,
        # and MSP always starts
        self.set_parameters({
            "OSD_TYPE": 5,
            "OSD_TYPE2": 3,
        })
        self.reboot_sitl()
        self.wait_statustext("MSP DisplayPort uart not available", check_context=True)
        self.wait_ready_to_arm()

        # TXONLY never creates a backend
        self.set_parameter("OSD_TYPE", 4)
        self.reboot_sitl()
        self.wait_ready_to_arm()

        # a second OSD refused as incompatible with the first still fails pre-arm
        self.set_parameters({
            "OSD_TYPE": 3,
            "OSD_TYPE2": 5,
        })
        self.reboot_sitl()
        self.assert_prearm_failure("OSD_TYPE2 not compatible with first OSD",
                                   other_prearm_failures_fatal=False)
