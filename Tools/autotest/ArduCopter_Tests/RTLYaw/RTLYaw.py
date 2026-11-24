'''
Copter autotest: RTLYaw

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RTLYaw(TestGroup):
    @testcase(bucket="CopterTests2a")
    def RTLYaw(self):
        '''test that vehicle yaws to original heading on RTL'''
        # the vehicle's heading is whatever the previous test left it
        # at - reboot to get the known spawn heading
        self.reboot_sitl()
        # 0 is WP_YAW_BEHAVIOR_NONE
        # 1 is WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP
        # 2 is WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL
        # 3 is WP_YAW_BEHAVIOR_LOOK_AHEAD
        for behaviour in 1, 3:
            self.set_parameters({
                'WP_YAW_BEHAVIOR': behaviour,
            })
            self.change_mode('GUIDED')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            original_heading = self.get_heading()
            target_heading = 100
            if original_heading - target_heading < 90:
                raise NotAchievedException("Bad initial heading")
            self.takeoff(10, mode='GUIDED')
            self.guided_achieve_heading(target_heading, direction=1, accuracy=1)
            self.change_mode('RTL')
            self.wait_heading(original_heading)
            self.wait_disarmed()
