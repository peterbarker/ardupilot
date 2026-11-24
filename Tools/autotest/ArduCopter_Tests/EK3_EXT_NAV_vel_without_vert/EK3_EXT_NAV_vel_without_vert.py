'''
Copter autotest: EK3_EXT_NAV_vel_without_vert

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class EK3_EXT_NAV_vel_without_vert(TestGroup):
    @testcase(bucket="CopterTests1a")
    def EK3_EXT_NAV_vel_without_vert(self):
        '''Test that EK3 External Navigation velocity works without vertical velocity.'''

        # Setup Vicon system and EKF3 parameters
        self.set_parameters({
            "VISO_TYPE": 2,      # Enable Vicon system
            "SERIAL5_PROTOCOL": 2,
            "EK3_ENABLE": 1,      # Enable EKF3
            "EK3_SRC2_POSXY": 6,  # External Navigation for horizontal position
            "EK3_SRC2_POSZ": 1,   # Use Barometer for altitude
            "EK3_SRC2_VELXY": 6,  # External Navigation for horizontal velocity
            "EK3_SRC2_VELZ": 0,   # No vertical velocity input
            "EK3_SRC2_YAW": 1,    # Use Compass for yaw
            "EK3_SRC_OPTIONS": 0,  # Dont fuse all velocity sources
            "RC8_OPTION": 90,     # RC aux switch 8 set to EKF source selector
            "AHRS_EKF_TYPE": 3,   # Enable EKF3
        })

        # Customize SITL command line for Vicon simulation
        self.customise_SITL_commandline(["--serial5=sim:vicon:"])

        # Switch to use Vicon
        self.set_rc(8, 1500)

        # Take off
        self.progress("Moving")
        self.takeoffAndMoveAway()

        # Increase Vicon vertical velocity error
        self.set_parameter("SIM_VICON_VGLI_Z", 1)

        # Set innovation limit
        innov_limit = 0.1  # Threshold for velocity variance

        max_variance = 0.0
        tstart = self.get_sim_time()
        while True:
            msg = self.assert_receive_message(type="EKF_STATUS_REPORT")
            variance = msg.velocity_variance
            max_variance = max(max_variance, variance)
            if variance > innov_limit:
                raise NotAchievedException(
                    f"Velocity variance too high {variance:.2f} > {innov_limit:.2f}"
                )
            if self.get_sim_time_cached() - tstart > 10:
                break
        print(f"Max variance before enabling vertical velocity fusion: {max_variance:.2f}")

        # Now enable vertical velocity fusion and check for innovation spike
        self.set_parameter("EK3_SRC2_VELZ", 6)

        spike_detected = False
        max_variance = 0.0
        tstart = self.get_sim_time()
        while True:
            msg = self.assert_receive_message(type="EKF_STATUS_REPORT")
            variance = msg.velocity_variance
            max_variance = max(max_variance, variance)
            if variance > innov_limit:
                spike_detected = True
            if self.get_sim_time_cached() - tstart > 10:
                break
        print(f"Max variance after enabling vertical velocity fusion: {max_variance:.2f}")

        if not spike_detected:
            raise NotAchievedException("Velocity variance did not spike as expected after enabling vertical velocity fusion.")

        self.disarm_vehicle(force=True)
