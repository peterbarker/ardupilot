'''
Copter autotest: AHRSOriginRecorded

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AHRSOriginRecorded(TestGroup):
    @testcase(bucket="CopterTests2b")
    def AHRSOriginRecorded(self):
        """Test AHRS option to record and reuse origin"""
        self.context_push()

        # The firmware writes these itself once the origin is known, so
        # register them now, while they are still as the session started;
        # otherwise the origin recorded here is left behind for every test
        # which follows.
        self.context_preserve_parameters([
            'AHRS_ORIGIN_LAT', 'AHRS_ORIGIN_LON', 'AHRS_ORIGIN_ALT',
        ])
        # Set AHRS_OPTIONS = 8 (UseRecordedOrigin)
        self.set_parameter('AHRS_OPTIONS', 8)
        self.set_parameter('LOG_DISARMED', 1)

        # AP_AHRS records the origin on the transition to having one
        # ("if (origin_ok && !state.origin_ok)"), so if an earlier test in
        # this session has already given the EKF an origin then the edge
        # never comes again and nothing is written.  Reboot so that the
        # transition happens with the option above already set.
        self.reboot_sitl()

        # wait for vehicle to be ready to arm which means origin should have been written
        self.wait_ready_to_arm()

        # confirm the AHRS_ORIG_LAT, LNG, ALT parameters are non-zero
        self.assert_parameter_value('AHRS_OPTIONS', 8)
        self.assert_parameter_value('AHRS_ORIGIN_LAT', -35, epsilon=1)
        self.assert_parameter_value('AHRS_ORIGIN_LON', 149, epsilon=1)
        self.assert_parameter_value('AHRS_ORIGIN_ALT', 584, epsilon=10)

        # enable optical flow and rangefinder
        self.set_parameter("SIM_FLOW_ENABLE", 1)
        self.set_parameter("FLOW_TYPE", 10)
        self.set_analog_rangefinder_parameters()
        self.configure_EKFs_to_use_optical_flow_instead_of_GPS()
        self.reboot_sitl()

        # wait for origin to be set
        self.wait_statustext("EKF3 IMU0 origin set")

        # switch to Loiter, arm, takeoff and RTL
        self.change_mode("LOITER")
        self.wait_ready_to_arm(timeout=120, require_absolute=False, check_prearm_bit=True)
        self.arm_vehicle()
        self.takeoff(10, mode="LOITER")
        self.do_RTL()

        # restore params to original values
        self.context_pop()
