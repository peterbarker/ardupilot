'''
Copter autotest: WPYawBehaviour1RTL

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class WPYawBehaviour1RTL(TestGroup):
    @testcase(bucket="CopterTests1a")
    def WPYawBehaviour1RTL(self):
        '''ensure behaviour 1 (face home) works in RTL'''
        self.start_subtest("moving off in guided mode and checking return yaw")
        # the vehicle's heading is whatever the previous test left it
        # at - reboot to get the known spawn heading
        self.reboot_sitl()
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.wait_heading(272, timeout=1)  # verify initial heading"
        self.takeoff(2, mode='GUIDED')
        self.set_parameter('WP_YAW_BEHAVIOR', 1)  # 1 is face next waypoint
        self.fly_guided_move_local(100, 100, z_up=20)
        self.wait_heading(45, timeout=1)
        self.change_mode('RTL')
        self.wait_heading(225, minimum_duration=10, timeout=20)
        self.wait_disarmed()

        self.reboot_sitl()
        self.start_subtest("now the same but in auto mode")
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 2),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 100, 100, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.set_parameter('AUTO_OPTIONS', 3)
        self.change_mode('AUTO')
        self.set_rc(3, 1000)
        self.wait_ready_to_arm()
        self.wait_heading(273, timeout=1)  # verify initial heading"
        self.arm_vehicle()
        self.wait_heading(45, minimum_duration=10, timeout=20)
        self.wait_current_waypoint(3)
        self.wait_heading(225, minimum_duration=10, timeout=20)
        self.wait_disarmed()
