'''
Rover autotest: AutoModeAccelLimiting

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AutoModeAccelLimiting(TestGroup):
    @testcase(bucket="Rover")
    def AutoModeAccelLimiting(self):
        '''verify ATC_ACCEL_MAX and ATC_DECEL_MAX cap WP_ACCEL'''

        high_speed = 6.0  # m/s
        low_speed  = 2.0  # m/s

        self.start_subtest("ATC_ACCEL_MAX limits WP_ACCEL from standing start")
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1000, 0, 0),
        ])
        wp_accel_high = 3.0   # m/s^2 — intentionally above the cap value
        atc_accel_cap = 0.5   # m/s^2
        meas_start    = 1.0   # m/s
        meas_stop     = 3.0   # m/s
        self.set_parameters({
            "WP_SPEED":      high_speed,
            "WP_ACCEL":      wp_accel_high,
            "ATC_ACCEL_MAX": 20.0,
            "ATC_DECEL_MAX": 5.0,
        })
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_groundspeed(meas_start, 100, timeout=30)
        tstart = self.get_sim_time()
        self.wait_groundspeed(meas_stop, 100, timeout=15)
        time_uncapped = self.get_sim_time() - tstart
        self.progress("ATC_ACCEL_MAX uncapped ramp %.2fs" % time_uncapped)
        self.change_mode('HOLD')
        self.wait_groundspeed(0, 0.5, timeout=30)
        self.set_parameter("ATC_ACCEL_MAX", atc_accel_cap)
        self.change_mode('AUTO')
        self.wait_groundspeed(meas_start, 100, timeout=30)
        tstart = self.get_sim_time()
        self.wait_groundspeed(meas_stop, 100, timeout=30)
        time_capped = self.get_sim_time() - tstart
        self.progress("ATC_ACCEL_MAX=%.1f capped ramp %.2fs" % (atc_accel_cap, time_capped))
        if time_capped < time_uncapped * 2.0:
            raise NotAchievedException(
                "ATC_ACCEL_MAX=%.1f did not limit WP_ACCEL as expected "
                "(uncapped=%.2fs capped=%.2fs)" %
                (atc_accel_cap, time_uncapped, time_capped))
        self.disarm_vehicle(force=True)

        self.start_subtest("ATC_DECEL_MAX limits WP_ACCEL via mid-leg WP_SPEED drop")
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  600, 0, 0),  # WP1
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1200, 0, 0),  # WP2
        ])
        decel_trigger = high_speed - 0.7   # 5.3 m/s — confirm at cruise before triggering
        decel_from    = high_speed - 1.0   # 5.0 m/s
        decel_to      = 3.0               # m/s — 2 m/s band
        atc_decel_cap = 0.5               # m/s^2
        self.set_parameters({
            "WP_SPEED":      high_speed,
            "ATC_ACCEL_MAX": 20.0,
            "ATC_DECEL_MAX": 0.0,
        })
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_groundspeed(decel_trigger, 100, timeout=30)
        self.set_parameter("WP_SPEED", low_speed)
        self.wait_groundspeed(0, decel_from, timeout=30)
        tstart = self.get_sim_time()
        self.wait_groundspeed(0, decel_to, timeout=30)
        time_uncapped_decel = self.get_sim_time() - tstart
        self.progress("ATC_DECEL_MAX uncapped decel %.2fs" % time_uncapped_decel)
        self.set_parameters({
            "WP_SPEED":      high_speed,
            "ATC_DECEL_MAX": atc_decel_cap,
        })
        self.wait_current_waypoint(2, timeout=120)
        self.wait_groundspeed(decel_trigger, 100, timeout=30)
        self.set_parameter("WP_SPEED", low_speed)
        self.wait_groundspeed(0, decel_from, timeout=30)
        tstart = self.get_sim_time()
        self.wait_groundspeed(0, decel_to, timeout=60)
        time_capped_decel = self.get_sim_time() - tstart
        self.progress("ATC_DECEL_MAX=%.1f capped decel %.2fs" % (atc_decel_cap, time_capped_decel))
        if time_capped_decel < time_uncapped_decel * 1.5:
            raise NotAchievedException(
                "ATC_DECEL_MAX=%.1f did not limit decel rate "
                "(uncapped=%.2fs capped=%.2fs)" %
                (atc_decel_cap, time_uncapped_decel, time_capped_decel))
        self.disarm_vehicle(force=True)
