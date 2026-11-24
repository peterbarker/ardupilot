'''
Copter autotest: Ch6TuningWPSpeed

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class Ch6TuningWPSpeed(TestGroup):
    @testcase(bucket="CopterTests1e")
    def Ch6TuningWPSpeed(self):
        '''test waypoint speed can be changed via Ch6 tuning knob'''
        self.set_parameters({
            "RC6_OPTION": 219,  # RC6 used for tuning
            "TUNE": 61,  # 61 is waypoint speed (m/s)
            "TUNE_MIN": 0.2, # 0.2m/s
            "TUNE_MAX": 10,  # 10m/s
            "AUTO_OPTIONS": 3,
        })
        self.set_rc(6, 2000)
        self.reboot_sitl()

        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.change_mode('AUTO')

        self.wait_ready_to_arm()

        self.arm_vehicle()

        self.wait_groundspeed(9.5, 10.5, minimum_duration=5)

        self.set_rc(6, 1500)
        self.wait_groundspeed(4.5, 5.5, minimum_duration=5)

        self.set_rc(6, 2000)
        self.wait_groundspeed(9.5, 10.5, minimum_duration=5)

        self.set_rc(6, 1300)
        self.wait_groundspeed(2.5, 3.5, minimum_duration=5)

        self.do_RTL()
