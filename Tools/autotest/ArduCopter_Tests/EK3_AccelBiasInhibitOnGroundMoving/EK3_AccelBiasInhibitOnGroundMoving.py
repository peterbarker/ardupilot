'''
Copter autotest: EK3_AccelBiasInhibitOnGroundMoving

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class EK3_AccelBiasInhibitOnGroundMoving(TestGroup):
    @testcase(bucket="CopterTests1c")
    def EK3_AccelBiasInhibitOnGroundMoving(self):
        '''Test EKF3 inhibits accel bias learning when vehicle is moved on ground'''
        # When a copter is on the ground and being moved (carried, on a ship),
        # the EKF should not learn accel biases from the external motion.
        # Without the fix (onGroundNotMoving check in dvelBiasAxisInhibit),
        # the Z-axis bias is learnable on the ground (passes gravity alignment
        # check), and GPS velocity innovations can drive incorrect bias learning
        # during ground movement. With the fix, all bias learning is inhibited
        # when onGround && !onGroundNotMoving.

        # Enable logging while disarmed so we capture ground-only EKF data
        self.set_parameters({
            "LOG_FILE_DSRMROT": 1,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()

        # Phase 1: Positive control - verify bias learning works when stationary
        self.start_subtest("Positive control: verify bias learning works when stationary")
        self.set_parameters({
            'SIM_ACC2_BIAS_Z': 0.7,
        })
        self.delay_sim_time(30, reason="EKF to learn accel bias")
        self.assert_dataflash_message_field_level_at(
            "XKF2", "AZ", 0.7,
            condition="XKF2.C==1",
            maintain=1,
        )

        # Reset for next phase - reboot clears EKF state and starts a new log
        self.set_parameters({
            'SIM_ACC2_BIAS_Z': 0.0,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.delay_sim_time(5, reason="SITL to initialise after reboot")

        # Phase 2: Start ship movement, then inject bias - it should NOT be learned
        self.start_subtest("Test: verify bias NOT learned during ground movement")
        # Use a small circle to create sufficient centripetal acceleration and
        # yaw rate so that onGroundNotMoving transitions to false.
        # v=10 m/s, path_size=50m (r=25m) gives yaw_rate=0.4 rad/s which
        # exceeds the gyro movement threshold.
        self.set_parameters({
            "SIM_SHIP_ENABLE": 1,
            "SIM_SHIP_SPEED": 10,
            "SIM_SHIP_PSIZE": 50,
            "SIM_SHIP_DSIZE": 10,
        })
        # Wait for vehicle to be moving with the ship
        self.wait_groundspeed(9, 11)
        # Allow time for onGroundNotMoving to transition to false
        self.delay_sim_time(5, reason="onGroundNotMoving to transition")

        # Record timestamp, then inject Z-bias while vehicle is moving on ground
        tstart_inject_us = self.get_sim_time() * 1.0e6
        self.set_parameters({
            'SIM_ACC2_BIAS_Z': 0.7,
        })

        # Wait for potential (unwanted) bias learning
        self.delay_sim_time(30, reason="potential bias learning period")

        # Check that AZ stayed near 0 during the movement period.
        # Skip the first 20s after injection to give time for any erroneous
        # learning to manifest. Then require AZ near 0 for 5 consecutive seconds.
        # With fix: AZ stays near 0 (learning inhibited) -> PASSES
        # Without fix: AZ drifts toward 0.7 -> FAILS
        tstart_check_us = tstart_inject_us + 20.0e6
        self.assert_dataflash_message_field_level_at(
            "XKF2", "AZ", 0.0,
            condition="XKF2.C==1",
            maintain=5,
            tolerance=0.15,
            dfreader_start_timestamp=tstart_check_us,
        )

        # Phase 3: Stop ship, verify bias learning resumes when stationary.
        # After inhibition ends, the saved (small) variance is restored, so
        # reconvergence is slow. Allow extra time and accept partial convergence.
        self.start_subtest("Verify bias learning resumes after movement stops")
        self.set_parameters({
            "SIM_SHIP_ENABLE": 0,
        })
        self.wait_groundspeed(0, 2)

        self.delay_sim_time(60, reason="accel bias to reconverge after movement stops")
        self.assert_dataflash_message_field_level_at(
            "XKF2", "AZ", 0.7,
            condition="XKF2.C==1",
            maintain=1,
            tolerance=0.3,
        )
