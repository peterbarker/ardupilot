'''
Copter autotest: Weathervane

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class Weathervane(TestGroup):
    @testcase(bucket="CopterTests2c")
    def Weathervane(self):
        '''Test copter weathervaning'''
        # We test nose into wind code paths and yaw direction here and test side into wind
        # yaw direction in QuadPlane tests to reduce repetition.
        self.set_parameters({
            "SIM_WIND_SPD": 10,
            "SIM_WIND_DIR": 100,
            "GUID_OPTIONS": 129, # allow weathervaning and arming from tx in guided
            "AUTO_OPTIONS": 131, # allow arming in auto, take off without raising the stick, and weathervaning
            "WVANE_ENABLE": 1,
            "WVANE_GAIN": 3,
            "WVANE_VELZ_MAX": 1,
            "WVANE_SPD_MAX": 2,
            # the roll deadzone leaves the heading settled short of the wind
            "WVANE_ANG_MIN": 0,
        })

        self.progress("Test weathervaning in auto")
        self.load_mission("weathervane_mission.txt", strict=False)

        self.change_mode("AUTO")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.wait_statustext("Weathervane Active", timeout=60)
        self.do_RTL()
        self.wait_disarmed()
        self.change_mode("GUIDED")

        # After take off command in guided we enter the velaccl sub mode
        self.progress("Test weathervaning in guided vel-accel")
        self.set_rc(3, 1000)
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.user_takeoff(alt_min=15)
        # Wait for heading to match wind direction.
        self.wait_heading(100, accuracy=5, minimum_duration=10, timeout=100)

        self.progress("Test weathervaning in guided pos only")
        # Travel directly north to align heading north and build some airspeed.
        self.fly_guided_move_local(x=40, y=0, z_up=15)
        # Wait for heading to match wind direction.
        self.wait_heading(100, accuracy=5, minimum_duration=10, timeout=100)
        self.do_RTL()
