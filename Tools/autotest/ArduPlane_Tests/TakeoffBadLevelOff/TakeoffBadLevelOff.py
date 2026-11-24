'''
Plane autotest: TakeoffBadLevelOff

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class TakeoffBadLevelOff(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def TakeoffBadLevelOff(self):
        '''Ensure that the takeoff can be completed under 0 pitch demand.'''
        '''
        When using no airspeed, the pitch level-off will eventually command 0
        pitch demand. Ensure that the plane can climb the final 2m to deem the
        takeoff complete.
        '''

        self.customise_SITL_commandline(
            [],
            model='plane-catapult',
        )
        self.set_parameters({
            "ARSPD_USE": 0.0,
            "PTCH_TRIM_DEG": -10.0,
            "RTL_AUTOLAND": 2, # The mission contains a DO_LAND_START item.
            "TKOFF_ALT": 50.0,
            "TKOFF_DIST": 1000.0,
            "TKOFF_THR_MAX": 75.0,
            "TKOFF_THR_MINACC": 3.0,
        })

        self.start_flying_mission("flaps_tkoff_50.txt")

        # Throw the catapult.
        self.set_servo(7, 2000)

        # Wait until we've reached the takeoff altitude.
        target_alt = 50
        self.wait_altitude(target_alt-1, target_alt+1, relative=True, timeout=30)

        self.delay_sim_time(5, reason="altitude to settle")

        self.disarm_vehicle(force=True)
