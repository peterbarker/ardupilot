'''
Copter autotest: MT11MAVFTP32bit

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MT11MAVFTP32bit(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MT11MAVFTP32bit(self):
        """Camera discovery, telemetry and FTP with wide vehicle and GCS IDs."""
        old_source = self.mav.source_system
        old_sysid = self.sysid_thismav()
        self.send_set_parameter_direct("MAV_SYSID", 100000)
        self.mav.target_system = 100000
        with self.mavlink_target_system_context():
            try:
                self.wait_heartbeat(timeout=60)
                self.mav.source_system = 70000
                self.mav.mav.srcSystem = 70000
                self.MT11MAVFTP()
            finally:
                self.mav.source_system = old_source
                self.mav.mav.srcSystem = old_source
                self.send_set_parameter_direct("MAV_SYSID", old_sysid)
                self.mav.target_system = old_sysid
                self.wait_heartbeat(timeout=60)
