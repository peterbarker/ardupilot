'''
Plane autotest: AirspeedCal

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AirspeedCal(TestGroup):
    def trigger_airspeed_cal(self):
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p3=1,
        )

    @testcase(bucket="PlaneTests1b")
    def AirspeedCal(self):
        '''test Airspeed calibration'''

        self.start_subtest('1 airspeed sensor')
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.trigger_airspeed_cal()
        self.wait_statustext('Airspeed 1 calibrated', check_context=True)
        self.context_pop()

        self.context_push()
        self.context_collect('STATUSTEXT')
        self.start_subtest('0 airspeed sensors')
        self.set_parameter('ARSPD_TYPE', 0)
        self.reboot_sitl()
        self.wait_statustext('No airspeed sensor', check_context=True)
        self.trigger_airspeed_cal()
        self.delay_sim_time(5, reason="airspeed calibration check period")
        if self.statustext_in_collections('Airspeed 1 calibrated'):
            raise NotAchievedException("Did not disable airspeed sensor?!")
        self.context_pop()

        self.start_subtest('2 airspeed sensors')
        self.set_parameter('ARSPD_TYPE', 100)
        self.set_parameter('ARSPD2_TYPE', 100)
        self.reboot_sitl()
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.trigger_airspeed_cal()
        self.wait_statustext('Airspeed 1 calibrated', check_context=True)
        self.wait_statustext('Airspeed 2 calibrated', check_context=True)
        self.context_pop()

        self.reboot_sitl()
