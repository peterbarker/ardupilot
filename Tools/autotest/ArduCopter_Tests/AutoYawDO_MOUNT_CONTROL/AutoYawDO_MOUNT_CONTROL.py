'''
Copter autotest: AutoYawDO_MOUNT_CONTROL

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class AutoYawDO_MOUNT_CONTROL(TestGroup):
    @testcase(bucket="CopterTests1d")
    def AutoYawDO_MOUNT_CONTROL(self):
        '''test AutoYaw behaviour when MAV_CMD_DO_MOUNT_CONTROL sent to Mount without Yaw control'''

        # setup mount parameters
        self.context_push()

        yaw_servo = 7
        self.setup_servo_mount(roll_servo=5, pitch_servo=6, yaw_servo=yaw_servo)
        # Disable Mount Yaw servo
        self.set_parameters({
            "SERVO%u_FUNCTION" % yaw_servo: 0,
        })
        self.reboot_sitl() # to handle MNT_TYPE changing

        self.takeoff(20, mode='GUIDED')

        for mount_yaw in [-45, 0, 45]:
            heading = 330
            self.guided_achieve_heading(heading, minimum_duration=1, direction=1)
            self.assert_heading(heading)

            self.neutralise_gimbal()

            r = 15
            p = 20
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                p1=p,
                p2=r,
                p3=mount_yaw,
                p7=mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
            )
            self.wait_heading(heading + mount_yaw, accuracy=1, minimum_duration=2)
            # We have disabled yaw servo, so expect mount yaw to be zero
            self.assert_mount_rpy(r, p, 0)
            # But we expect the copter to yaw instead
            self.assert_heading(heading + mount_yaw)

        self.do_RTL()

        self.context_pop()
        self.reboot_sitl()
