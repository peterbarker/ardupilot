'''
Copter autotest: Scripting6DoFMotors

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class Scripting6DoFMotors(TestGroup):
    @testcase(bucket="CopterTests2c")
    def Scripting6DoFMotors(self):
        '''test 6DoF scripting motor matrix'''
        self.context_collect('STATUSTEXT')

        self.set_parameters({
            "FRAME_CLASS": 16,  # MOTOR_FRAME_6DOF_SCRIPTING
            "SCR_ENABLE": 1,
        })

        self.install_example_script_context("Copter_Motors_6DoF.lua")
        self.reboot_sitl()

        self.wait_statustext("6DoF Copter quad scripting", timeout=30, check_context=True)
        self.wait_ready_to_arm()

        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 30, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 30, 30, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        num_wp = self.get_mission_count()
        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.arm_vehicle()
        self.wait_waypoint(num_wp-1, num_wp-1, timeout=120)
        self.wait_disarmed(timeout=60)
