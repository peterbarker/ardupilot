'''
Rover autotest: WheelEncoders

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class WheelEncoders(TestGroup):
    @testcase(bucket="Rover")
    def WheelEncoders(self):
        '''make sure wheel encoders are generally working'''
        self.set_parameters({
            "WENC_TYPE": 10,
            "EK3_ENABLE": 1,
            "AHRS_EKF_TYPE": 3,
        })
        self.reboot_sitl()
        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.change_mode("MANUAL")
        self.arm_vehicle()
        self.set_rc(3, 1600)

        m = self.assert_receive_message('WHEEL_DISTANCE', timeout=5)

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                break
            dist_home = self.distance_to_home(use_cached_home=True)
            m = self.mav.messages.get("WHEEL_DISTANCE")
            delta = abs(m.distance[0] - dist_home)
            self.progress("dist-home=%f wheel-distance=%f delta=%f" %
                          (dist_home, m.distance[0], delta))
            if delta > 5:
                raise NotAchievedException("wheel distance incorrect")
        self.disarm_vehicle()
