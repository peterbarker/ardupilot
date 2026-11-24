'''
Copter autotest: GCSFailsafe

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GCSFailsafe(TestGroup):
    # Tests all actions and logic behind the GCS failsafe
    @testcase(bucket="CopterTests1d")
    def GCSFailsafe(self, side=60, timeout=360):
        '''Test GCS Failsafe'''
        # Test double-SmartRTL; ensure we do SmarRTL twice rather than
        # landing (tests fix for actual bug)
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)
        self.context_push()
        self.start_subtest("GCS failsafe SmartRTL twice")
        self.setGCSfailsafe(3)
        self.set_parameter('FS_OPTIONS', 8)
        self.takeoffAndMoveAway()
        self.set_heartbeat_rate(0)
        self.wait_mode("SMART_RTL")
        self.wait_disarmed()
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)

        self.takeoffAndMoveAway()
        self.set_heartbeat_rate(0)
        self.wait_statustext("GCS Failsafe")

        def ensure_smartrtl(mav, m):
            if m.get_type() != "HEARTBEAT":
                return
            # can't use mode_is here because we're in the message hook
            print("Mode: %s" % self.mav.flightmode)
            if self.mav.flightmode != "SMART_RTL":
                raise NotAchievedException("Not in SMART_RTL")
        self.install_message_hook_context(ensure_smartrtl)

        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.set_heartbeat_rate(0)
        self.wait_statustext("GCS Failsafe")

        self.wait_disarmed()

        self.end_subtest("GCS failsafe SmartRTL twice")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.context_pop()

        # Trigger telemetry loss with failsafe disabled. Verify no action taken.
        self.start_subtest("GCS failsafe disabled test: FS_GCS_ENABLE=0 should take no failsafe action")
        self.setGCSfailsafe(0)
        self.takeoffAndMoveAway()
        self.set_heartbeat_rate(0)
        self.delay_sim_time(5, reason="GCS failsafe to not trigger")
        self.wait_mode("ALT_HOLD")
        self.set_heartbeat_rate(self.speedup)
        self.delay_sim_time(5, reason="GCS link to restore")
        self.wait_mode("ALT_HOLD")
        self.end_subtest("Completed GCS failsafe disabled test")

        # Trigger telemetry loss with failsafe enabled. Verify
        # failsafe triggers to RTL. Restore telemetry, verify failsafe
        # clears, and change modes.
        self.start_subtest("GCS failsafe recovery test: FS_GCS_ENABLE=1 & FS_OPTIONS=0")
        self.setGCSfailsafe(1)
        self.set_parameter('FS_OPTIONS', 0)
        self.set_heartbeat_rate(0)
        self.wait_mode("RTL")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.change_mode("LOITER")
        self.end_subtest("Completed GCS failsafe recovery test")

        # Trigger telemetry loss with failsafe enabled. Verify
        # failsafe triggers to RTL. Restore telemetry, verify failsafe
        # clears, and change modes.
        self.start_subtest("GCS failsafe recovery test: FS_GCS_ENABLE=1 & FS_OPTIONS=0 & FS_GCS_TIMEOUT=10")
        self.setGCSfailsafe(1)
        self.set_parameter('FS_OPTIONS', 0)
        old_gcs_timeout = self.get_parameter("FS_GCS_TIMEOUT")
        new_gcs_timeout = old_gcs_timeout * 2
        self.set_parameter("FS_GCS_TIMEOUT", new_gcs_timeout)
        self.set_heartbeat_rate(0)
        self.delay_sim_time(old_gcs_timeout + (new_gcs_timeout - old_gcs_timeout) / 2, reason="GCS failsafe timeout to elapse")
        self.assert_mode("LOITER")
        self.wait_mode("RTL")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.change_mode("LOITER")
        self.set_parameter('FS_GCS_TIMEOUT', old_gcs_timeout)
        self.end_subtest("Completed GCS failsafe recovery test")

        # Trigger telemetry loss with failsafe enabled. Verify failsafe triggers and RTL completes
        self.start_subtest("GCS failsafe RTL with no options test: FS_GCS_ENABLE=1 & FS_OPTIONS=0")
        self.setGCSfailsafe(1)
        self.set_parameter('FS_OPTIONS', 0)
        self.set_heartbeat_rate(0)
        self.wait_mode("RTL")
        self.wait_rtl_complete()
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe RTL with no options test")

        # Trigger telemetry loss with failsafe enabled. Verify failsafe triggers and land completes
        self.start_subtest("GCS failsafe LAND with no options test: FS_GCS_ENABLE=5 & FS_OPTIONS=0")
        self.setGCSfailsafe(5)
        self.takeoffAndMoveAway()
        self.set_heartbeat_rate(0)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe land with no options test")

        # Trigger telemetry loss with failsafe enabled. Verify failsafe triggers and SmartRTL completes
        self.start_subtest("GCS failsafe SmartRTL->RTL with no options test: FS_GCS_ENABLE=3 & FS_OPTIONS=0")
        self.setGCSfailsafe(3)
        self.takeoffAndMoveAway()
        self.set_heartbeat_rate(0)
        self.wait_mode("SMART_RTL")
        self.wait_disarmed()
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe SmartRTL->RTL with no options test")

        # Trigger telemetry loss with failsafe enabled. Verify failsafe triggers and SmartRTL completes
        self.start_subtest("GCS failsafe SmartRTL->Land with no options test: FS_GCS_ENABLE=4 & FS_OPTIONS=0")
        self.setGCSfailsafe(4)
        self.takeoffAndMoveAway()
        self.set_heartbeat_rate(0)
        self.wait_mode("SMART_RTL")
        self.wait_disarmed()
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe SmartRTL->Land with no options test")

        # Trigger telemetry loss with an invalid failsafe value. Verify failsafe triggers and RTL completes
        self.start_subtest("GCS failsafe invalid value with no options test: FS_GCS_ENABLE=99 & FS_OPTIONS=0")
        self.setGCSfailsafe(99)
        self.takeoffAndMoveAway()
        self.set_heartbeat_rate(0)
        self.wait_mode("RTL")
        self.wait_rtl_complete()
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe invalid value with no options test")

        # Trigger telemetry loss with failsafe enabled to test FS_OPTIONS settings
        self.start_subtest("GCS failsafe with option bit tests: FS_GCS_ENABLE=1 & FS_OPTIONS=64/2/16")
        num_wp = self.load_mission("copter_mission.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load copter_mission failed")
        self.setGCSfailsafe(1)
        self.set_parameter('FS_OPTIONS', 16)
        self.takeoffAndMoveAway()
        self.progress("Testing continue in pilot controlled modes")
        self.set_heartbeat_rate(0)
        self.wait_statustext("GCS Failsafe - Continuing Pilot Control", timeout=60)
        self.delay_sim_time(5, reason="GCS failsafe to not trigger")
        self.wait_mode("ALT_HOLD")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)

        self.progress("Testing continue in auto mission")
        self.set_parameter('FS_OPTIONS', 2)
        self.change_mode("AUTO")
        self.delay_sim_time(5, reason="auto mode to start")
        self.set_heartbeat_rate(0)
        self.wait_statustext("GCS Failsafe - Continuing Auto Mode", timeout=60)
        self.delay_sim_time(5, reason="GCS failsafe to not trigger")
        self.wait_mode("AUTO")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)

        self.progress("Testing continue landing in land mode")
        self.set_parameter('FS_OPTIONS', 8)
        self.change_mode("LAND")
        self.delay_sim_time(5, reason="land mode to start")
        self.set_heartbeat_rate(0)
        self.wait_statustext("GCS Failsafe - Continuing Landing", timeout=60)
        self.delay_sim_time(5, reason="GCS failsafe to not trigger")
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.end_subtest("Completed GCS failsafe with option bits")

        self.setGCSfailsafe(0)
        self.set_parameter('FS_OPTIONS', 0)
        self.progress("All GCS failsafe tests complete")

        # we are not at the home location - reboot so the next test starts there
        self.reboot_sitl()
