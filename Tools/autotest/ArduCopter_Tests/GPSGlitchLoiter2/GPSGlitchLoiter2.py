'''
Copter autotest: GPSGlitchLoiter2

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSGlitchLoiter2(TestGroup):
    def gps_glitch_loiter2_for_ekf_type(self, active_type):
        '''run the GPSGlitchLoiter2 sequence with the given EKF active'''
        self.context_push()
        self.set_parameters({
            "AHRS_EKF_TYPE": active_type,
            "EK2_ENABLE": 1,
            "EK3_ENABLE": 1,
        })
        self.reboot_sitl()
        self.takeoff(10, mode="LOITER")

        # wait for vehicle to level
        self.wait_attitude(desroll=0, despitch=0, timeout=10, tolerance=1)

        # apply glitch
        self.set_parameter("SIM_GPS1_GLTCH_X", 0.001)

        # check lean angles remain stable for 20 seconds
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 20:
            m = self.assert_receive_message('ATTITUDE')
            roll_deg = math.degrees(m.roll)
            pitch_deg = math.degrees(m.pitch)
            self.progress("checking att: roll=%f pitch=%f " % (roll_deg, pitch_deg))
            if abs(roll_deg) > 2 or abs(pitch_deg) > 2:
                raise NotAchievedException("fly_gps_glitch_loiter_test2 failed, roll or pitch moved during GPS glitch")

        # RTL; the context pop removes the glitch and reboots
        self.do_RTL(alt_max=2)
        self.context_pop()

    @testcase(bucket="CopterTests1d")
    def GPSGlitchLoiter2(self):
        """test vehicle handles GPS glitch (aka EKF Reset) without twitching"""
        # both enabled filters see glitch-induced position resets; the
        # vehicle response only depends on the active filter, so fly
        # once with each:
        for active_type in 3, 2:
            self.start_subtest("active EKF type %u" % active_type)
            self.gps_glitch_loiter2_for_ekf_type(active_type)
