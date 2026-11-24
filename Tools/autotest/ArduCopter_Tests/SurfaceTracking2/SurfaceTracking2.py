'''
Copter autotest: SurfaceTracking2

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SurfaceTracking2(TestGroup):
    @testcase(bucket="CopterTests1a")
    def SurfaceTracking2(self):
        """test the AP_SurfaceDistance library parameters"""

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

        self.start_subtest("Testing normal glitch procedure.")
        threshold = self.get_parameter("SURFTRAK_GLDST")
        # Glitch it below the threshold, observe that the reading is not rejected and there is no reset.
        self.set_parameter("SIM_SONAR_OFFSET", -(threshold-glitch_offset))
        self.delay_sim_time(5, "wait for the vehicle altitude to stabilize")
        self.assert_altitude(alt_initial + (threshold - glitch_offset), accuracy=0.5)
        restore_offset(10)
        self.delay_sim_time(5, "wait for the vehicle altitude to stabilize")
        self.assert_altitude(alt_initial, accuracy=0.5)

        # Glitch it over the threshold, observe there is reset.
        self.set_parameter("SIM_SONAR_OFFSET", -(threshold+glitch_offset))
        self.delay_sim_time(5, "wait for the vehicle altitude to stabilize")
        # Vehicle doesn't change altitude, because the rangefinder offset has been reset.
        self.assert_altitude(alt_initial, accuracy=0.5)
        restore_offset(10)
        self.delay_sim_time(5, "wait for the vehicle altitude to stabilize")
        self.assert_altitude(alt_initial - (threshold + glitch_offset), accuracy=0.5)

        self.start_subtest("Testing that the glitch threshold can be extended.")
        alt_initial = self.get_altitude()
        # Increase the threshold.
        # The threshold is extended by glitch_offset+1, which means that if the parameter doesn't work,
        # both cases below would trigger a reset.
        self.set_parameter("SURFTRAK_GLDST", threshold+glitch_offset+1)
        threshold = self.get_parameter("SURFTRAK_GLDST")
        # Glitch it below the threshold, observe that the reading is not rejected and there is no reset.
        self.set_parameter("SIM_SONAR_OFFSET", -(threshold-glitch_offset))
        self.delay_sim_time(5, "wait for the vehicle altitude to stabilize")
        self.assert_altitude(alt_initial + (threshold - glitch_offset), accuracy=0.5)
        restore_offset(10)
        self.delay_sim_time(5, "wait for the vehicle to stabilize again")
        self.assert_altitude(alt_initial, accuracy=0.5)

        # Glitch it over the threshold, observe there is reset.
        self.set_parameter("SIM_SONAR_OFFSET", -(threshold+glitch_offset))
        self.delay_sim_time(5, "wait for the vehicle altitude to stabilize")
        self.assert_altitude(alt_initial, accuracy=0.5)
        restore_offset(10)
        self.delay_sim_time(5, "wait for the vehicle to stabilize again")
        self.assert_altitude(alt_initial - (threshold + glitch_offset), accuracy=0.5)

        self.start_subtest("Testing that the glitch reset can be disabled.")
        alt_initial = self.get_altitude()
        self.set_parameter("SURFTRAK_GLSAM", 0)  # Disable resetting.
        # Glitch it above the threshold, observe that the reading is rejected and there is no reset.
        self.set_parameter("SIM_SONAR_OFFSET", -(threshold+glitch_offset))
        self.delay_sim_time(5, "wait for the vehicle altitude to stabilize")
        self.assert_altitude(alt_initial, accuracy=0.5)  # The aircraft should reject the measurements and not climb.
        restore_offset(10)
        self.delay_sim_time(5, "wait for the vehicle to stabilize again")
        self.assert_altitude(alt_initial, accuracy=0.5)  # The aircraft should have maintained the same altitude reference.

        self.do_RTL()
