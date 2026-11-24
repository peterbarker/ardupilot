'''
Plane autotest: AIRSPEED_AUTOCAL

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AIRSPEED_AUTOCAL(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def AIRSPEED_AUTOCAL(self):
        '''Test AIRSPEED_AUTOCAL'''
        self.progress("Ensure no AIRSPEED_AUTOCAL on ground")
        self.set_parameters({
            "ARSPD_AUTOCAL": 1,
            "ARSPD_PIN": 2,
            "ARSPD_RATIO": 0,
            "ARSPD2_RATIO": 4,
            "ARSPD2_TYPE": 3,  # MS5525
            "ARSPD2_BUS": 1,
            "ARSPD2_AUTOCAL": 1,
            "SIM_ARSPD2_OFS": 1900,  # default is 2013

            "RTL_AUTOLAND": 1,
        })
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()

        self.assert_not_receive_message('AIRSPEED_AUTOCAL', timeout=5)

        # these are boot-time calibration messages:
        self.wait_statustext('Airspeed 1 calibrated', check_context=True, timeout=30)
        self.wait_statustext('Airspeed 2 calibrated', check_context=True)

        mission_filepath = "flaps.txt"
        self.load_mission(mission_filepath)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode("AUTO")
        self.progress("Ensure AIRSPEED_AUTOCAL in air")
        self.assert_receive_message('AIRSPEED_AUTOCAL')
        self.wait_statustext("Airspeed 0 ratio reset", check_context=True, timeout=70)
        self.wait_statustext("Airspeed 1 ratio reset", check_context=True, timeout=70)
        self.fly_home_land_and_disarm()
