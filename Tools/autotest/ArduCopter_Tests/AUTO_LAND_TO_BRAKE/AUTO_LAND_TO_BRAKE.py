'''
Copter autotest: AUTO_LAND_TO_BRAKE

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AUTO_LAND_TO_BRAKE(TestGroup):
    @testcase(bucket="CopterTests1c")
    def AUTO_LAND_TO_BRAKE(self):
        '''ensure terrain altitude is taken into account when braking'''
        self.set_parameters({
            "PLND_ACC_P_NSE": 2.500000,
            "PLND_ALT_MAX": 8.000000,
            "PLND_ALT_MIN": 0.750000,
            "PLND_BUS": -1,
            "PLND_CAM_POS_X": 0.000000,
            "PLND_CAM_POS_Y": 0.000000,
            "PLND_CAM_POS_Z": 0.000000,
            "PLND_ENABLED": 1,
            "PLND_EST_TYPE": 1,
            "PLND_LAG": 0.020000,
            "PLND_LAND_OFS_X": 0.000000,
            "PLND_LAND_OFS_Y": 0.000000,
            "PLND_OPTIONS": 0,
            "PLND_RET_BEHAVE": 0,
            "PLND_RET_MAX": 4,
            "PLND_STRICT": 1,
            "PLND_TIMEOUT": 4.000000,
            "PLND_TYPE": 4,
            "PLND_XY_DIST_MAX": 2.500000,
            "PLND_YAW_ALIGN": 0.000000,

            "SIM_PLD_ALT_LMT": 15.000000,
            "SIM_PLD_DIST_LMT": 10.000000,
            "SIM_PLD_ENABLE": 1,
            "SIM_PLD_HEIGHT": 0,
            "SIM_PLD_LAT": -20.558929,
            "SIM_PLD_LON": -47.415035,
            "SIM_PLD_RATE": 100,
            "SIM_PLD_TYPE": 1,
            "SIM_PLD_YAW": 87,

            "SIM_SONAR_SCALE": 12,
        })

        self.set_analog_rangefinder_parameters()

        self.load_mission('mission.txt')
        self.customise_SITL_commandline([
            "--home", self.sitl_home_string_from_mission("mission.txt"),
        ])

        self.set_parameter('AUTO_OPTIONS', 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.wait_current_waypoint(7)
        self.wait_altitude(10, 15, relative=True, timeout=60)
        self.change_mode('BRAKE')
        # monitor altitude here
        self.wait_altitude(10, 15, relative=True, minimum_duration=20)
        self.change_mode('AUTO')
        self.wait_disarmed()
