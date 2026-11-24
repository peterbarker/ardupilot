'''
Rover autotest: BeaconPosition

AP_FLAKE8_CLEAN
'''

import vehicle_test_suite

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase

from rover import SITL_START_LOCATION


class BeaconPosition(TestGroup):
    @testcase(bucket="Rover")
    def BeaconPosition(self):
        '''drive in MANUAL using only a (simulated) beacon for position'''
        self.set_parameters({
            "BCN_TYPE": 10,    # SITL
            "BCN_LATITUDE": SITL_START_LOCATION.lat,
            "BCN_LONGITUDE": SITL_START_LOCATION.lng,
            "BCN_ALT": SITL_START_LOCATION.get_alt_m(AltFrame.ABSOLUTE),
            "BCN_ORIENT_YAW": 0,
            "GPS1_TYPE": 0,    # no GPS
            "EK3_ENABLE": 1,
            "EK3_SRC1_POSXY": 4,  # Beacon
            "EK3_SRC1_VELXY": 0,  # None
            "EK3_SRC1_POSZ": 1,   # Baro
            "EK3_SRC1_VELZ": 0,   # None
            "EK3_SRC1_YAW": 1,    # Compass
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
        })
        self.reboot_sitl()

        # becoming armable with the GPS disabled proves the EKF is deriving a
        # usable position purely from the beacon library, which the vehicle
        # initialises and updates via AP_Vehicle:
        # (require_absolute=False as we have deliberately disabled the GPS)
        self.wait_ready_to_arm(require_absolute=False)

        # use get_location() (GLOBAL_POSITION_INT, the EKF/beacon-fused
        # position) rather than pymavlink's mavfile.location(), which blocks
        # waiting for a GPS 3D fix that never arrives with the GPS disabled:
        start_loc = self.get_location()
        self.progress("Beacon-derived start location: %s" % str(start_loc))

        # arm and drive forward in MANUAL, confirming we can travel a known
        # distance using only the beacon for position.  Throughout the drive,
        # validate that the EKF's reported position (GLOBAL_POSITION_INT) stays
        # close to the true simulator position (SIMSTATE):
        self.context_push()
        self.install_message_hook_context(
            vehicle_test_suite.TestSuite.ValidateGlobalPositionIntAgainstSimState(
                self, max_allowed_divergence=5))
        # wait for the beacon-derived position to agree with the true one
        # before driving.  Polling change_mode() and armed() get us here
        # sooner in simulated time than they used to, and the EKF's
        # position is still converging: the divergence hook above then
        # tripped on the residual, at 5.27-5.41m against its 5m limit.
        # This is a condition rather than a delay, so a position which
        # never settles fails here instead of mid-drive.
        self.wait_simstate_divergence_below(2, minimum_duration=5, timeout=60)
        self.change_mode('MANUAL')
        self.arm_vehicle()
        self.set_rc(3, 2000)   # full throttle forward
        self.wait_distance(10, accuracy=2, timeout=60)
        self.set_rc(3, 1500)   # stop
        # hold station and confirm the beacon-derived position stays locked to
        # the true position (GLOBAL_POSITION_INT vs SIMSTATE) while stationary:
        self.delay_sim_time(30, reason="stationary beacon position tracking")
        self.disarm_vehicle()
        self.context_pop()

        self.assert_current_onboard_log_contains_message("BCN")
