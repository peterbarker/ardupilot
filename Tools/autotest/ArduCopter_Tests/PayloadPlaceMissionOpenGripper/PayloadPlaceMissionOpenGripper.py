'''
Copter autotest: PayloadPlaceMissionOpenGripper

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PayloadPlaceMissionOpenGripper(TestGroup):
    @testcase(bucket="CopterTests2d")
    def PayloadPlaceMissionOpenGripper(self):
        '''test running the mission when the gripper is open'''
        self.set_analog_rangefinder_parameters()
        self.set_servo_gripper_parameters()
        self.reboot_sitl()

        self.load_mission("copter_payload_place.txt")

        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()

        self.progress("Opening the gripper before time")
        self.run_auxfunc(19, 0)

        self.wait_text("Abort: Gripper Open", timeout=90)

        self.wait_disarmed()
