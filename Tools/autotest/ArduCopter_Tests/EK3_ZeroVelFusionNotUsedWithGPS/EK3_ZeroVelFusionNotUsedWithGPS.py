'''
Copter autotest: EK3_ZeroVelFusionNotUsedWithGPS

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class EK3_ZeroVelFusionNotUsedWithGPS(TestGroup):
    @testcase(bucket="CopterTests1c")
    def EK3_ZeroVelFusionNotUsedWithGPS(self):
        '''Test EKF3 zero velocity changes do not affect GPS-enabled setups'''
        # Addresses review concern: does zero velocity fusion interfere
        # with GPS-enabled configurations? This test places the vehicle on
        # a fast-moving boat with GPS active and verifies:
        # 1. EKF tracks GPS velocity (not pulled to zero)
        # 2. Accel bias learning works via GPS velocity observations
        #
        # The ship uses a large circle (1000m diameter) at 15 m/s so
        # motion is smooth enough that onGroundNotMoving stays TRUE once
        # the ship reaches steady speed. Despite this, zero velocity
        # fusion does NOT activate because GPS provides recent velocity
        # data (haveRecentGpsVel is true in SelectVelPosFusion), so the
        # condition for synthetic zero velocity injection is never met.

        self.set_parameters({
            "LOG_FILE_DSRMROT": 1,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()

        # Start ship with large radius to keep IMU motion below the
        # on-ground-not-moving detection thresholds. At 15 m/s on
        # 1000m diameter (r=500m) with EK3_OGNM_TEST_SF=2.0 (default):
        #   angular rate = 0.03 rad/s (1.7 deg/s) vs 6 deg/s threshold
        #   centripetal accel = 0.45 m/s^2 vs 2.0 m/s^2 threshold
        # So onGroundNotMoving remains TRUE at steady state.
        self.start_subtest("Verify EKF tracks GPS velocity on moving boat")
        self.set_parameters({
            "SIM_SHIP_ENABLE": 1,
            "SIM_SHIP_SPEED": 15,
            "SIM_SHIP_PSIZE": 1000,
            "SIM_SHIP_DSIZE": 10,
        })

        # Wait for ship to reach speed and movement filters to settle.
        # During initial acceleration onGroundNotMoving briefly goes
        # false, but recovers once at constant speed.
        self.wait_groundspeed(13, 17)
        self.delay_sim_time(10, reason="movement filters to settle")

        # Verify EKF velocity matches GPS, not zero. If zero velocity
        # fusion were incorrectly active, groundspeed would drift to 0.
        self.wait_groundspeed(13, 17, timeout=10)

        # Inject Z-bias and verify bias learning via GPS velocity.
        # With onGroundNotMoving TRUE and gravity-aligned Z axis, the
        # bias is observable. GPS velocity provides the Kalman filter
        # measurement needed to converge the bias estimate.
        self.start_subtest("Verify accel bias learned via GPS velocity")
        self.set_parameters({
            'SIM_ACC2_BIAS_Z': 0.7,
        })
        self.delay_sim_time(30, reason="EKF to learn accel bias via GPS velocity")
        self.assert_dataflash_message_field_level_at(
            "XKF2", "AZ", 0.7,
            condition="XKF2.C==1",
            maintain=1,
        )

        # Confirm velocity still tracking GPS after bias convergence
        self.wait_groundspeed(13, 17, timeout=10)
