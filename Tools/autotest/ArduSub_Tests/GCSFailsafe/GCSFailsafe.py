'''
Sub autotest: GCSFailsafe

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GCSFailsafe(TestGroup):
    @testcase(bucket="Sub")
    def GCSFailsafe(self):
        '''Test GCSFailsafe'''
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)
        self.setGCSfailsafe(4)
        self.set_heartbeat_rate(0)
        self.wait_mode("SURFACE")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.progress("GSC Failsafe OK")
        self.disarm_vehicle()
