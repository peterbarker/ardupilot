'''
Sub autotest: UpsideDown

AP_FLAKE8_CLEAN
'''

from math import radians

from pymavlink import mavextra
from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class UpsideDown(TestGroup):
    @testcase(bucket="Sub")
    def UpsideDown(self):
        """Test roll authority and attitude control of vectored_6dof frame via SET_ATTITUDE_TARGET."""
        model = "vectored_6dof"
        self.customise_SITL_commandline(
            [],
            model=model,
            defaults_filepath=self.model_defaults_filepath('vectored_6dof'),
        )

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('ALT_HOLD')

        type_mask = (
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE |
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE |
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE |
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE
        )

        def command_and_wait_roll(target_deg, timeout=30, hold_seconds=2):
            target_quat = mavextra.euler_to_quat([radians(target_deg), 0, 0])

            def cf(value, _target):
                self.mav.mav.set_attitude_target_send(
                    0,  # timestamp
                    self.sysid_thismav(), 1,  # target IDs
                    type_mask,
                    target_quat,
                    0, 0, 0,  # (ignored) attitude rate targets
                    0.5  # thrust
                )
            self.wait_roll(
                target_deg,
                accuracy=5,
                timeout=timeout,
                absolute_value=True,
                minimum_duration=hold_seconds,
                called_function=cf,
            )

        for roll_deg in [45, 90, 120, 180]:
            self.start_subtest(f"Roll {roll_deg} degrees")
            command_and_wait_roll(roll_deg)

            self.start_subtest(f"Return to level from {roll_deg} degrees")
            command_and_wait_roll(0)

        self.disarm_vehicle()
