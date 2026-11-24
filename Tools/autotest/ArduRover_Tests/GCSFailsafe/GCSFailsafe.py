'''
Rover autotest: GCSFailsafe

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GCSFailsafe(TestGroup):
    @testcase(bucket="Rover")
    def GCSFailsafe(self, side=60, timeout=360):
        """Test GCS Failsafe"""
        try:
            self.test_gcs_failsafe(side=side, timeout=timeout)
        except Exception as ex:
            self.setGCSfailsafe(0)
            self.set_parameter('FS_ACTION', 0)
            self.disarm_vehicle(force=True)
            self.reboot_sitl()
            raise ex

    def test_gcs_failsafe(self, side=60, timeout=360):
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)
        self.set_parameter("FS_ACTION", 1)
        self.set_parameter("FS_THR_ENABLE", 0)  # disable radio FS as it inhibt GCS one's

        def go_somewhere():
            self.change_mode("MANUAL")
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.set_rc(3, 2000)
            self.delay_sim_time(5, reason="vehicle to move")
            self.set_rc(3, 1500)
        # Trigger telemetry loss with failsafe disabled. Verify no action taken.
        self.start_subtest("GCS failsafe disabled test: FS_GCS_ENABLE=0 should take no failsafe action")
        self.setGCSfailsafe(0)
        go_somewhere()
        self.set_heartbeat_rate(0)
        self.delay_sim_time(5, reason="GCS failsafe timeout")
        self.wait_mode("MANUAL")
        self.set_heartbeat_rate(self.speedup)
        self.delay_sim_time(5, reason="heartbeat restoration")
        self.wait_mode("MANUAL")
        self.end_subtest("Completed GCS failsafe disabled test")

        # Trigger telemetry loss with failsafe enabled. Verify
        # failsafe triggers to RTL. Restore telemetry, verify failsafe
        # clears, and change modes.
        self.start_subtest("GCS failsafe recovery test: FS_GCS_ENABLE=1")
        self.setGCSfailsafe(1)
        go_somewhere()
        self.set_heartbeat_rate(0)
        self.wait_mode("RTL")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.change_mode("MANUAL")
        self.end_subtest("Completed GCS failsafe recovery test")

        # Trigger telemetry loss with failsafe enabled. Verify failsafe triggers and RTL completes
        self.start_subtest("GCS failsafe RTL with no options test: FS_GCS_ENABLE=1")
        self.setGCSfailsafe(1)
        self.set_heartbeat_rate(0)
        self.wait_mode("RTL")
        self.wait_statustext("Reached destination", timeout=60)
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe RTL")

        # Trigger telemetry loss with an invalid failsafe value. Verify failsafe triggers and RTL completes
        self.start_subtest("GCS failsafe invalid value with no options test: FS_GCS_ENABLE=99")
        self.setGCSfailsafe(99)
        go_somewhere()
        self.set_heartbeat_rate(0)
        self.wait_mode("RTL")
        self.wait_statustext("Reached destination", timeout=60)
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe invalid value")

        self.start_subtest("Testing continue in auto mission")
        self.disarm_vehicle()
        self.setGCSfailsafe(2)
        self.load_mission("test_arming.txt")
        self.change_mode("AUTO")
        self.delay_sim_time(5, reason="mission waypoints to start")
        self.set_heartbeat_rate(0)
        self.wait_statustext("Failsafe - Continuing Auto Mode", timeout=60)
        self.delay_sim_time(5, reason="failsafe continuation check")
        self.wait_mode("AUTO")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)

        self.start_subtest("GCS failsafe RTL with no options test: FS_GCS_ENABLE=1 and FS_GCS_TIMEOUT=10")
        self.setGCSfailsafe(1)
        old_gcs_timeout = self.get_parameter("FS_GCS_TIMEOUT")
        new_gcs_timeout = old_gcs_timeout * 2
        self.set_parameter("FS_GCS_TIMEOUT", new_gcs_timeout)
        go_somewhere()
        self.set_heartbeat_rate(0)
        self.delay_sim_time(old_gcs_timeout + (new_gcs_timeout - old_gcs_timeout) / 2, reason="GCS timeout midpoint")
        self.assert_mode("MANUAL")
        self.wait_mode("RTL")
        self.wait_statustext("Reached destination", timeout=60)
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.disarm_vehicle()
        self.end_subtest("Completed GCS failsafe RTL")

        self.setGCSfailsafe(0)
        self.progress("All GCS failsafe tests complete")
