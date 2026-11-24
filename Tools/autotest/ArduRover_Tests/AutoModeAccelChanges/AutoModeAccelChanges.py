'''
Rover autotest: AutoModeAccelChanges

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AutoModeAccelChanges(TestGroup):
    @testcase(bucket="Rover")
    def AutoModeAccelChanges(self):
        '''verify WP_ACCEL mid-mission change bakes at the next WP crossing and takes effect n+2 legs later'''
        # Change WP_ACCEL during WP1->WP2; bakes at WP2 crossing;
        # WP2->WP3 may still propagate old value; WP3->WP4 is the reliable measurement leg.
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  300,    0, 0),  # WP1
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  300,  300, 0),  # WP2
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  600,  300, 0),  # WP3
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  600,  600, 0),  # WP4
        ])
        wp_accel    = 0.5   # m/s^2: slow enough for a clear ~2.0 s measurement
        accel_start = 4.0   # m/s
        accel_stop  = 5.0   # m/s
        high_speed  = 6.0   # m/s
        self.set_parameters({
            "WP_SPEED":      high_speed,
            "WP_ACCEL":      3.0,    # initial fast rate; changed mid-mission below
            "ATC_ACCEL_MAX": 20.0,
        })
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')

        self.wait_current_waypoint(2, timeout=120)     # rover is now on WP1->WP2
        self.set_parameter("WP_ACCEL", wp_accel)       # bakes at WP2 crossing

        expected_time = (accel_stop - accel_start) / wp_accel  # 2.0 s
        self.wait_current_waypoint(4, timeout=120)     # WP3->WP4: first reliable leg
        self.wait_groundspeed(0, accel_start - 0.5, timeout=30)  # wait for WP3 corner dip
        self.wait_groundspeed(accel_start, 100, timeout=30)
        tstart = self.get_sim_time()
        self.wait_groundspeed(accel_stop, 100, timeout=expected_time * 3)
        actual_time = self.get_sim_time() - tstart
        if actual_time > expected_time * 2.0:
            raise NotAchievedException(
                "WP_ACCEL=%.1f too slow: expected ~%.1fs, got %.1fs" %
                (wp_accel, expected_time, actual_time))
        if actual_time < expected_time * 0.3:
            raise NotAchievedException(
                "WP_ACCEL=%.1f too fast: expected ~%.1fs, got %.1fs" %
                (wp_accel, expected_time, actual_time))
        self.progress("WP_ACCEL=%.1f ramp time %.1fs (expected ~%.1fs)" %
                      (wp_accel, actual_time, expected_time))
        self.disarm_vehicle(force=True)
