'''
Copter autotest: SurfaceTrackingCornerCases

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SurfaceTrackingCornerCases(TestGroup):
    @testcase(bucket="CopterTests1e")
    def SurfaceTrackingCornerCases(self):
        """test the AP_SurfaceDistance library behaviour under real-life corner-cases"""

        def restore_offset(steps):
            """Gradually restore SIM_SONAR_OFFSET back to 0."""
            initial_value = self.get_parameter("SIM_SONAR_OFFSET")
            for i in range(steps):
                self.set_parameter("SIM_SONAR_OFFSET", initial_value*(steps-1-i)/steps)

        self.set_analog_rangefinder_parameters()
        self.reboot_sitl()

        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.takeoff(15, takeoff_throttle=1700, mode="LOITER")
        alt_initial = self.get_altitude()
        glitch_offset = 1
        # Test cases of real-life.
        self.start_subtest("Fly over building and come back.")
        # This is the same as the first test, but made explicit for the real-life use-case.
        # Assumption: The building is lower than the glitch distance threshold.
        threshold = self.get_parameter("SURFTRAK_GLDST")
        # Fly over the building, clipping the side-wall abruptly, the rangefinder reading reduces.
        self.set_parameter("SIM_SONAR_OFFSET", -(threshold-glitch_offset))
        # This should result in increase of altitude.
        target = alt_initial + (threshold - glitch_offset)
        self.wait_altitude(target-0.5, target+0.5, minimum_duration=5, timeout=20)
        # Fly off the building. The vertical wall is scanned by the beam, resulting in gradual reading increase.
        restore_offset(10)
        # We should be once again at the original altitude.
        self.wait_altitude(alt_initial-0.5, alt_initial+0.5, minimum_duration=5, timeout=20)

        self.start_subtest("Fly with interference from an underslung tether.")
        threshold = self.get_parameter("SURFTRAK_GLDST")
        # Disable reference resets, we expect a lot of false glitches, due to cable interference.
        self.set_parameter("SURFTRAK_GLSAM", 0)
        # The tether moves in the FOV of the sensor.
        self.set_parameter("SIM_SONAR_OFFSET", -(threshold+glitch_offset))
        # The vehicle should reject the reading and keep still.
        self.wait_altitude(alt_initial-0.5, alt_initial+0.5, minimum_duration=5, timeout=20)
        # The tether moves out of the FOV of the sensor.
        self.set_parameter("SIM_SONAR_OFFSET", 0)
        # We should still be at the same altitude.
        self.wait_altitude(alt_initial-0.5, alt_initial+0.5, minimum_duration=5, timeout=20)

        self.do_RTL()
