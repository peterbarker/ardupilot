'''
Plane autotest: LandingDrift

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class LandingDrift(TestGroup):
    @testcase(bucket="PlaneTests1b", disabled="Flapping test. See https://github.com/ArduPilot/ardupilot/issues/20054")
    def LandingDrift(self):
        '''Circuit with baro drift'''
        self.customise_SITL_commandline([], wipe=True)

        self.set_analog_rangefinder_parameters()

        self.set_parameters({
            "SIM_BARO_DRIFT": -0.02,
            "SIM_TERRAIN": 0,
            "RNGFND_LANDING": 1,
            "LAND_SLOPE_RCALC": 2,
            "LAND_ABORT_DEG": 1,
        })

        self.reboot_sitl()

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Load and start mission
        self.load_mission("ap-circuit.txt", strict=True)
        self.set_current_waypoint(1, check_afterwards=True)
        self.change_mode('AUTO')
        self.wait_current_waypoint(1, timeout=5)
        self.wait_groundspeed(0, 10, timeout=5)

        # Wait for landing waypoint
        self.wait_current_waypoint(9, timeout=1200)

        # Wait for landing restart
        self.wait_current_waypoint(5, timeout=60)

        # Wait for landing waypoint (second attempt)
        self.wait_current_waypoint(9, timeout=1200)

        self.wait_disarmed(timeout=180)
