'''
Sub autotest: GuidedAboveTerrain

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GuidedAboveTerrain(TestGroup):
    @testcase(bucket="Sub")
    def GuidedAboveTerrain(self):
        """Test Guided_PosVelAccel mode and position offsets via Lua"""
        self.context_collect("STATUSTEXT")

        seafloor_depth = 15
        match_distance = 2    # Target distance from seafloor (HAGL)
        xy_margin = 1         # Allowed error horizontal
        z_margin = 2          # Allowed error vertical

        # Install the GAT PVA script
        # This will send rapid updates to Guided_PosVelAccel and poscontrol
        self.install_example_script_context("guided_above_terrain_posvelaccel_sub.lua")

        # Install the synthetic seafloor rangefinder script and reboot
        self.prepare_synthetic_seafloor_test(seafloor_depth, match_distance)

        # Set the desired forward speed
        speed = 0.5
        self.set_parameter('GAT_SPEED', speed)

        # Minimize vertical oscillation due to sensor delay
        self.set_parameter('PSC_D_JERK', 4.0)
        self.set_parameter('WP_ACC_Z', 5.0)

        self.dive(-seafloor_depth + match_distance, mode="ALT_HOLD")

        start_loc = self.get_location()
        timeout = 60

        # Activate the GAT script
        self.change_mode("GUIDED")
        self.wait_text("GAT: active", check_context=True)

        # Verify the sub maintains the target distance over the terrain
        self.watch_true_distance_maintained(match_distance, delta=z_margin, timeout=timeout)

        # Verify that we moved the expected distance
        end_loc = self.get_location()
        distance_travelled = self.get_distance(start_loc, end_loc)
        expected_distance = speed * timeout
        self.progress(f"Expected distance {expected_distance}m, got {distance_travelled}m")
        if abs(distance_travelled - expected_distance) > xy_margin:
            raise NotAchievedException("Failed to achieve distance")

        self.disarm_vehicle()
