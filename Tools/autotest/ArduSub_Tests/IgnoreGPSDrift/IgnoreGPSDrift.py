'''
Sub autotest: IgnoreGPSDrift

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase

from ardusub import Joystick


class IgnoreGPSDrift(TestGroup):
    @testcase(bucket="Sub")
    def IgnoreGPSDrift(self):
        """ Test GPS-DVL integration. DVL (VISO) should be able to filter the GPS noise.
        this tests switching over to VISO, running for a bit with no sensors, re-enabling viso,
        and finally surfacing, where we expect the GPS position will be used to fix the drift
        resulting of running with no positioning sensors."""

        self.customise_SITL_commandline(["--serial5=sim:vicon:"])

        # configure EKF to use external nav instead of GPS
        self.set_parameters({
            "EK3_SRC1_POSXY": 3,
            "EK3_SRC1_VELXY": 6,
            "EK3_SRC1_POSZ": 1,
            "EK3_SRC1_VELZ": 0,

            "VISO_TYPE": 1,
            "SERIAL5_PROTOCOL": 1,
            "SIM_VICON_TMASK": 8,  # send VISION_POSITION_DELTA
            # "SIM_VICON_TMASK": 2,  # send VISION_SPEED_ESTIMATE
            # "SIM_VICON_TMASK": 10,  # send VISION_SPEED_ESTIMATE AND VISION_POSITION_DELTA
            # "EK3_VELNE_M_NSE": 0.001,

            "GPS1_TYPE": 1,
            "SIM_GPS1_DRFTALT": 3,
            "SIM_GPS1_ACC": 0.3,
            "SIM_GPS1_ENABLE": 1,
            "SIM_GPS1_TYPE": 1,

            "GPS2_TYPE": 0,

            "EK3_POSNE_M_NSE": 100,
        })
        self.reboot_sitl()

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("Ensuring that VISO manages to filter the GPS noise")
        self.change_mode('POSHOLD')
        self.set_parameter("SIM_GPS1_HNSE", 2)
        self.wait_location(
            self.get_location('SIMSTATE'),
            location_source="SIMSTATE",
            minimum_duration=10,
            height_accuracy=None,  # SIMSTATE carries no altitude
            accuracy=1.0,
            timeout=60.0,
        )

        self.progress("Diving to 10m")
        self.set_rc(Joystick.Throttle, 1300)
        self.wait_altitude(altitude_min=-10, altitude_max=-9, relative=False, timeout=60)
        self.set_rc(Joystick.Throttle, 1500)

        self.progress("Disabling GPS")
        self.set_parameters({
            "SIM_GPS1_ENABLE": 0,
        })

        self.progress("Driving GPS-less for 10m using VISO")
        self.set_rc(Joystick.Forward, 1900)
        self.wait_distance(10, timeout=60, location_source="SIMSTATE")
        self.set_rc(Joystick.Forward, 1500)

        self.progress("Surfacing")
        self.set_rc(Joystick.Throttle, 1900)
        self.wait_altitude(altitude_min=-0.5, altitude_max=1, relative=False, timeout=60)
        self.set_rc(Joystick.Throttle, 1500)

        self.progress("Re-enabling GPS, expecting position to converge on the truth")
        self.set_parameters({
            "SIM_GPS1_ENABLE": 1,
        })
        # check the estimate settles near the vehicle's true position;
        # the noisy GPS (SIM_GPS1_HNSE=2, above) pulls it a couple of
        # metres away from the ExternalNav-only solution as it is fused
        # back in, so allow for that:
        self.wait_location(
            self.get_location('SIMSTATE'),
            minimum_duration=5,
            accuracy=4,
            height_accuracy=None,  # SIMSTATE carries no altitude
            timeout=60.0,
        )

        self.progress("Driving forward 10m with GPS+VISO")
        self.set_rc(Joystick.Forward, 1900)
        self.wait_distance(10, timeout=60, location_source="SIMSTATE")
        self.set_rc(Joystick.Forward, 1500)

        self.progress("Disabling GPS and diving to 10m for no-GPS+no-VISO test")
        self.set_parameters({
            "SIM_GPS1_ENABLE": 0,
        })
        self.set_rc(Joystick.Throttle, 1300)
        self.wait_altitude(altitude_min=-10, altitude_max=-9, relative=False, timeout=60)
        self.set_rc(Joystick.Throttle, 1500)

        self.progress("Driving 10m on VISO only, verifying EKF has relative position")
        self.set_rc(Joystick.Forward, 1900)
        self.wait_distance(10, timeout=60, location_source="SIMSTATE")
        self.wait_ekf_flags(
            mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL,
            0,
            timeout=15,
        )

        self.progress("Disabling VISO and waiting for EKF to lose position")
        self.set_parameters({
            "SIM_VICON_TMASK": 0,
        })
        self.wait_ekf_flags(
            0,
            mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL | mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS,
            timeout=15,
        )

        self.progress("Stopping to build position error while EKF has no aiding")
        self.set_rc(Joystick.Forward, 1500)
        self.delay_sim_time(10, reason="build position error while EKF has no aiding")

        self.progress("Re-enabling VISO and driving forward")
        self.set_rc(Joystick.Forward, 1900)
        self.delay_sim_time(5, reason="drive forward with no aiding")
        self.set_parameters({
            "SIM_VICON_TMASK": 10,
        })
        self.delay_sim_time(10, reason="drive forward with VISO aiding")
        self.set_rc(Joystick.Forward, 1500)

        self.progress("Surfacing to simulate re-acquiring GPS")
        self.set_rc(Joystick.Throttle, 1900)
        self.wait_altitude(altitude_min=-0.5, altitude_max=1, relative=False, timeout=60)
        self.set_rc(Joystick.Throttle, 1500)

        self.progress("Re-enabling GPS, expecting position correction")
        self.set_parameters({
            "SIM_GPS1_ENABLE": 1,
        })
        self.wait_distance(10, accuracy=5, timeout=60)
        self.disarm_vehicle()
