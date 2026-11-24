'''
Plane autotest: TakeoffLevelOffWind

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class TakeoffLevelOffWind(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def TakeoffLevelOffWind(self):
        '''Ensure the level-off functionality works.'''
        '''
        This is primarily targeted to test whether the level-off angle works
        correctly even though the groundspeed eventually drops in face of wind.
        '''
        tkoff_alt = 100.
        self.set_parameters({
            "TKOFF_ROTATE_SPD": 15.0,
            "TKOFF_ALT": tkoff_alt,
            "TKOFF_DIST": 500,  # Ensure stall prevention doesn't interfere with the test.
            "TKOFF_PLIM_SEC": 5  # Give some more time to detect the level-off.
        })
        self.change_mode("TAKEOFF")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Wait until we're well past the rotation.
        self.wait_groundspeed(24, 25)

        self.set_parameters({
            "SIM_WIND_DIR": 0.0,  # Set North wind.
            "SIM_WIND_SPD": 10.0  # Enough to bring groundspeed below cruise speed.
        })

        self.wait_altitude(tkoff_alt-10, tkoff_alt, relative=True)
        self.wait_level_flight(accuracy=10, timeout=1)  # Ensure we have roughly level-off.
        self.delay_sim_time(5, reason="level-off to confirm")

        self.change_mode('AUTOLAND')
        self.wait_disarmed(timeout=180)
