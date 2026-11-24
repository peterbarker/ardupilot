'''
Plane autotest: AltResetBadGPS

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AltResetBadGPS(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def AltResetBadGPS(self):
        '''Tests the handling of poor GPS lock pre-arm alt resets'''
        self.set_parameters({
            "SIM_GPS1_GLTCH_Z": 0,
            "SIM_GPS1_ACC": 0.3,
        })
        self.wait_ready_to_arm()

        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        relalt = m.relative_alt*0.001
        if abs(relalt) > 3:
            raise NotAchievedException("Bad relative alt %.1f" % relalt)

        self.progress("Setting low accuracy, glitching GPS")
        self.set_parameter("SIM_GPS1_ACC", 40)
        self.set_parameter("SIM_GPS1_GLTCH_Z", -47)

        self.progress("Waiting 10s for height update")
        self.delay_sim_time(10, reason="GPS altitude to update")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        relalt = m.relative_alt*0.001
        if abs(relalt) > 3:
            raise NotAchievedException("Bad glitching relative alt %.1f" % relalt)

        self.disarm_vehicle()
        # reboot to clear potentially bad state
