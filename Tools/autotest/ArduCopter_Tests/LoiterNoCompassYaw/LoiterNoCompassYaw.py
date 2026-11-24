'''
Copter autotest: LoiterNoCompassYaw

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class LoiterNoCompassYaw(TestGroup):
    @testcase(bucket="CopterTests1d")
    def LoiterNoCompassYaw(self):
        '''Loiter indoors with optical flow and no GPS, compass not an EK3 yaw source'''
        # Indoor case: position from optical flow + rangefinder, no GPS. The
        # compass stays enabled and healthy but is not an EK3 yaw source
        # (EK3_SRC1_YAW=0/None), so the EKF runs in relative-position mode with
        # a free-running yaw. using_noncompass_for_yaw() stays true, so the
        # Copter arm-time compass-health gate is skipped. Verify that arming
        # directly in Loiter is permitted and that Loiter stays engaged.
        self.set_parameters({
            "AHRS_EKF_TYPE": 3,
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 0,
            "EK3_SRC1_YAW": 0,  # None - compass enabled but not used for yaw
            "SIM_FLOW_ENABLE": 1,
            "FLOW_TYPE": 10,
            "SIM_GPS1_ENABLE": 0,
            "SIM_TERRAIN": 0,
        })
        self.configure_EKFs_to_use_optical_flow_instead_of_GPS()
        self.set_analog_rangefinder_parameters()
        self.reboot_sitl()

        self.wait_ready_to_arm(require_absolute=False, timeout=120)

        # arm and take off directly in Loiter: this is the path that matters -
        # Loiter requires_position(), and arming exercises the arm-time
        # compass-health gate while the compass is not the EKF yaw source.
        self.takeoff(altitude_min=10, mode='LOITER', require_absolute=False, takeoff_throttle=1800)

        # confirm Loiter stays engaged and armed (no EKF failsafe / mode
        # revert). A lat/lon position-hold check is not used: with no GPS the
        # EKF has no absolute origin, so global position is not valid here.
        self.delay_sim_time(15, "confirm Loiter stays engaged and armed")
        self.wait_mode('LOITER')
        if not self.armed():
            raise NotAchievedException("Disarmed during Loiter without compass yaw source")

        self.land_and_disarm()
