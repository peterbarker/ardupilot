'''
Copter autotest: BeaconPosition

AP_FLAKE8_CLEAN
'''

import vehicle_test_suite

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase

from arducopter import SITL_START_LOCATION


class BeaconPosition(TestGroup):
    def fly_beacon_position(self, old_pos):
        '''fly a beacon-position check against the currently-configured
        beacon backend; old_pos is a (GPS-derived) GLOBAL_POSITION_INT
        captured before the GPS was disabled'''
        # require_absolute=False as we have disabled the GPS:
        self.wait_ready_to_arm(require_absolute=False)

        tstart = self.get_sim_time()
        timeout = 30
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not get new position like old position")
            self.progress("Fetching location")
            new_pos = self.get_global_position_int()
            pos_delta = self.get_distance_int(old_pos, new_pos)
            max_delta = 2
            self.progress("delta=%u want <= %u" % (pos_delta, max_delta))
            if pos_delta <= max_delta:
                break

        self.progress("Moving to ensure location is tracked")
        self.takeoff(10, mode="STABILIZE")
        self.change_mode("CIRCLE")

        self.context_push()
        validator = vehicle_test_suite.TestSuite.ValidateGlobalPositionIntAgainstSimState(self, max_allowed_divergence=10)
        self.install_message_hook_context(validator)

        self.delay_sim_time(20, reason="position tracking validation")
        self.progress("Tracked location just fine")
        self.context_pop()

        self.change_mode("LOITER")
        self.wait_groundspeed(0, 0.3, timeout=120)
        self.land_and_disarm()

        self.assert_current_onboard_log_contains_message("BCN")

        self.disarm_vehicle(force=True)

    @testcase(bucket="CopterTests1a")
    def BeaconPosition(self):
        '''Fly Beacon Position over each beacon backend type'''
        self.reboot_sitl()

        self.wait_ready_to_arm(require_absolute=True)

        old_pos = self.get_global_position_int()
        print("old_pos=%s" % str(old_pos))

        # configuration common to all beacon backends:
        self.set_parameters({
            "BCN_LATITUDE": SITL_START_LOCATION.lat,
            "BCN_LONGITUDE": SITL_START_LOCATION.lng,
            "BCN_ALT": SITL_START_LOCATION.get_alt_m(AltFrame.ABSOLUTE),
            "BCN_ORIENT_YAW": 0,
            "AVOID_ENABLE": 4,
            "GPS1_TYPE": 0,
            "EK3_ENABLE": 1,
            "EK3_SRC1_POSXY": 4, # Beacon
            "EK3_SRC1_POSZ": 1,  # Baro
            "EK3_SRC1_VELXY": 0, # None
            "EK3_SRC1_VELZ": 0,  # None
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
        })

        # iterate over the beacon backends; each entry is the backend
        # name, the parameters which select it, and any extra SITL
        # command-line needed to attach a simulated device:
        backends = [
            ("SITL", {
                "BCN_TYPE": 10,  # SITL
            }, None),
            ("NoopLoop", {
                "BCN_TYPE": 3,           # NoopLoop
                "SERIAL5_PROTOCOL": 13,  # Beacon
            }, ["--serial5=sim:nooploop"]),
        ]
        for (name, params, customisations) in backends:
            self.start_subtest("Beacon backend: %s" % name)
            if customisations is not None:
                self.customise_SITL_commandline(customisations)
            self.set_parameters(params)
            self.reboot_sitl()
            self.fly_beacon_position(old_pos)
