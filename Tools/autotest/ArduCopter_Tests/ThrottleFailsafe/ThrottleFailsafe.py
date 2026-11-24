'''
Copter autotest: ThrottleFailsafe

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ThrottleFailsafe(TestGroup):
    # Tests all actions and logic behind the radio failsafe
    @testcase(bucket="CopterTests2b")
    def ThrottleFailsafe(self, side=60, timeout=360):
        '''Test Throttle Failsafe'''
        self.start_subtest("If you haven't taken off yet RC failure should be instant disarm")
        self.change_mode("STABILIZE")
        self.set_parameter("DISARM_DELAY", 0)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter("SIM_RC_FAIL", 1)
        self.disarm_wait(timeout=1)
        self.set_parameter("SIM_RC_FAIL", 0)
        self.set_parameter("DISARM_DELAY", 10)

        # Trigger an RC failure with the failsafe disabled. Verify no action taken.
        self.start_subtest("Radio failsafe disabled test: FS_THR_ENABLE=0 should take no failsafe action")
        self.set_parameter('FS_THR_ENABLE', 0)
        self.set_parameter('FS_OPTIONS', 0)
        self.takeoffAndMoveAway()
        self.set_parameter("SIM_RC_FAIL", 1)
        self.delay_sim_time(5, reason="radio failsafe to not trigger")
        self.wait_mode("ALT_HOLD")
        self.set_parameter("SIM_RC_FAIL", 0)
        self.delay_sim_time(5, reason="RC to restore")
        self.wait_mode("ALT_HOLD")
        self.end_subtest("Completed Radio failsafe disabled test")

        # Trigger an RC failure, verify radio failsafe triggers,
        # restore radio, verify RC function by changing modes to circle
        # and stabilize.
        self.start_subtest("Radio failsafe recovery test")
        self.set_parameter('FS_THR_ENABLE', 1)
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("RTL")
        self.delay_sim_time(5, reason="RTL to progress")
        self.set_parameter("SIM_RC_FAIL", 0)
        self.delay_sim_time(5, reason="RC to restore")
        self.set_rc(5, 1050)
        self.wait_mode("CIRCLE")
        self.set_rc(5, 1950)
        self.wait_mode("STABILIZE")
        self.end_subtest("Completed Radio failsafe recovery test")

        # Trigger and RC failure, verify failsafe triggers and RTL completes
        self.start_subtest("Radio failsafe RTL with no options test: FS_THR_ENABLE=1 & FS_OPTIONS=0")
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("RTL")
        self.wait_rtl_complete()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.end_subtest("Completed Radio failsafe RTL with no options test")

        # Trigger and RC failure, verify failsafe triggers and land completes
        self.start_subtest("Radio failsafe LAND with no options test: FS_THR_ENABLE=3 & FS_OPTIONS=0")
        self.set_parameter('FS_THR_ENABLE', 3)
        self.takeoffAndMoveAway()
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.end_subtest("Completed Radio failsafe LAND with no options test")

        # Trigger and RC failure, verify failsafe triggers and SmartRTL completes
        self.start_subtest("Radio failsafe SmartRTL->RTL with no options test: FS_THR_ENABLE=4 & FS_OPTIONS=0")
        self.set_parameter('FS_THR_ENABLE', 4)
        self.takeoffAndMoveAway()
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("SMART_RTL")
        self.wait_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.end_subtest("Completed Radio failsafe SmartRTL->RTL with no options test")

        # Trigger and RC failure, verify failsafe triggers and SmartRTL completes
        self.start_subtest("Radio failsafe SmartRTL->Land with no options test: FS_THR_ENABLE=5 & FS_OPTIONS=0")
        self.set_parameter('FS_THR_ENABLE', 5)
        self.takeoffAndMoveAway()
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("SMART_RTL")
        self.wait_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.end_subtest("Completed Radio failsafe SmartRTL_Land with no options test")

        # Trigger a GPS failure and RC failure, verify RTL fails into
        # land mode and completes
        self.start_subtest("Radio failsafe RTL fails into land mode due to bad position.")
        self.set_parameter('FS_THR_ENABLE', 1)
        self.takeoffAndMoveAway()
        self.set_parameter('SIM_GPS1_ENABLE', 0)
        self.delay_sim_time(5, reason="GPS disable to take effect")
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.set_parameter('SIM_GPS1_ENABLE', 1)
        self.wait_ekf_happy()
        self.end_subtest("Completed Radio failsafe RTL fails into land mode due to bad position.")

        # Trigger a GPS failure and RC failure, verify SmartRTL fails
        # into land mode and completes
        self.start_subtest("Radio failsafe SmartRTL->RTL fails into land mode due to bad position.")
        self.set_parameter('FS_THR_ENABLE', 4)
        self.takeoffAndMoveAway()
        self.set_parameter('SIM_GPS1_ENABLE', 0)
        self.delay_sim_time(5, reason="GPS disable to take effect")
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.set_parameter('SIM_GPS1_ENABLE', 1)
        self.wait_ekf_happy()
        self.end_subtest("Completed Radio failsafe SmartRTL->RTL fails into land mode due to bad position.")

        # Trigger a GPS failure and RC failure, verify SmartRTL fails
        # into land mode and completes
        self.start_subtest("Radio failsafe SmartRTL->LAND fails into land mode due to bad position.")
        self.set_parameter('FS_THR_ENABLE', 5)
        self.takeoffAndMoveAway()
        self.set_parameter('SIM_GPS1_ENABLE', 0)
        self.delay_sim_time(5, reason="GPS disable to take effect")
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.set_parameter('SIM_GPS1_ENABLE', 1)
        self.wait_ekf_happy()
        self.end_subtest("Completed Radio failsafe SmartRTL->LAND fails into land mode due to bad position.")

        # Trigger a GPS failure, then restore the GPS. Trigger an RC
        # failure, verify SmartRTL fails into RTL and completes
        self.start_subtest("Radio failsafe SmartRTL->RTL fails into RTL mode due to no path.")
        self.set_parameter('FS_THR_ENABLE', 4)
        self.takeoffAndMoveAway()
        self.set_parameter('SIM_GPS1_ENABLE', 0)
        self.wait_statustext("SmartRTL deactivated: bad position", timeout=60)
        self.set_parameter('SIM_GPS1_ENABLE', 1)
        self.wait_ekf_happy()
        self.delay_sim_time(5, reason="EKF to stabilise after GPS restore")
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("RTL")
        self.wait_rtl_complete()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.end_subtest("Completed Radio failsafe SmartRTL->RTL fails into RTL mode due to no path.")

        # Trigger a GPS failure, then restore the GPS. Trigger an RC
        # failure, verify SmartRTL fails into Land and completes
        self.start_subtest("Radio failsafe SmartRTL->LAND fails into land mode due to no path.")
        self.set_parameter('FS_THR_ENABLE', 5)
        self.takeoffAndMoveAway()
        self.set_parameter('SIM_GPS1_ENABLE', 0)
        self.wait_statustext("SmartRTL deactivated: bad position", timeout=60)
        self.set_parameter('SIM_GPS1_ENABLE', 1)
        self.wait_ekf_happy()
        self.delay_sim_time(5, reason="EKF to stabilise after GPS restore")
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("LAND")
        self.wait_landed_and_disarmed()
        self.set_parameter("SIM_RC_FAIL", 0)
        self.end_subtest("Completed Radio failsafe SmartRTL->LAND fails into land mode due to no path.")

        # Trigger an RC failure in guided mode with the option enabled
        # to continue in guided. Verify no failsafe action takes place
        self.start_subtest("Radio failsafe with option to continue in guided mode: FS_THR_ENABLE=1 & FS_OPTIONS=4")
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)
        self.set_parameter('FS_THR_ENABLE', 1)
        self.set_parameter('FS_OPTIONS', 4)
        self.takeoffAndMoveAway()
        # enable GCS failsafe only after the climb so the climb runs at
        # full speedup; it only needs to be armed before SIM_RC_FAIL
        self.setGCSfailsafe(1)
        self.change_mode("GUIDED")
        self.set_parameter("SIM_RC_FAIL", 1)
        self.delay_sim_time(5, reason="radio failsafe to not trigger")
        self.wait_mode("GUIDED")
        self.set_parameter("SIM_RC_FAIL", 0)
        self.delay_sim_time(5, reason="RC to restore")
        self.change_mode("ALT_HOLD")
        self.setGCSfailsafe(0)
        # self.change_mode("RTL")
        # self.wait_disarmed()
        self.end_subtest("Completed Radio failsafe with option to continue in guided mode")

        # Trigger an RC failure in AUTO mode with the option enabled
        # to continue the mission. Verify no failsafe action takes
        # place
        self.start_subtest("Radio failsafe RTL with option to continue mission: FS_THR_ENABLE=1 & FS_OPTIONS=1")
        self.set_parameter('FS_OPTIONS', 1)
        self.progress("# Load copter_mission")
        num_wp = self.load_mission("copter_mission.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load copter_mission failed")
#        self.takeoffAndMoveAway()
        self.change_mode("AUTO")
        self.set_parameter("SIM_RC_FAIL", 1)
        self.delay_sim_time(5, reason="radio failsafe to not trigger")
        self.wait_mode("AUTO")
        self.set_parameter("SIM_RC_FAIL", 0)
        self.delay_sim_time(5, reason="RC to restore")
        self.wait_mode("AUTO")
        # self.change_mode("RTL")
        # self.wait_disarmed()
        self.end_subtest("Completed Radio failsafe RTL with option to continue mission")

        # Trigger an RC failure in AUTO mode without the option
        # enabled to continue. Verify failsafe triggers and RTL
        # completes
        self.start_subtest("Radio failsafe RTL in mission without "
                           "option to continue should RTL: FS_THR_ENABLE=1 & FS_OPTIONS=0")
        self.set_parameter('FS_OPTIONS', 0)
        self.set_parameter("SIM_RC_FAIL", 1)
        self.wait_mode("RTL")
        self.wait_rtl_complete()
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        self.set_parameter("SIM_RC_FAIL", 0)
        self.end_subtest("Completed Radio failsafe RTL in mission without option to continue")

        self.progress("All radio failsafe tests complete")
        self.set_parameter('FS_THR_ENABLE', 0)
        self.reboot_sitl()
