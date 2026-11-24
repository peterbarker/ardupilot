'''
Copter autotest: GripperInitialPosition

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GripperInitialPosition(TestGroup):
    @testcase(bucket="CopterTests2c")
    def GripperInitialPosition(self):
        '''test gripper initial position'''

        self.set_servo_gripper_parameters()
        self.set_parameters({
            "RC8_OPTION": 19,  # gripper
        })
        grip_neutral = 1423
        self.set_parameter('GRIP_NEUTRAL', grip_neutral)

        # install a message hook which ensures that the output stays at neutral
        global seen_grip_neutral
        seen_grip_neutral = False

        def hook(mav, m):
            global seen_grip_neutral
            if m.get_type() != 'SERVO_OUTPUT_RAW':
                return
            if m.servo8_raw == 0:
                # initial value
                if seen_grip_neutral:
                    raise NotAchievedException("Saw 0 after seeing grip neutral")
                return
            if m.servo8_raw == grip_neutral:
                # expected value after autopilot boot
                seen_grip_neutral = True
                return
            raise NotAchievedException(f"Received bad value {m.servo8_raw=} for servo gripper")
        self.install_message_hook_context(hook)

        self.reboot_sitl()
        self.wait_ready_to_arm()

        if not seen_grip_neutral:
            raise NotAchievedException("Never saw grip neutral")
