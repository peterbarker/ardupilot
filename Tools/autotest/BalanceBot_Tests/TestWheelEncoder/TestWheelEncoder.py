'''
BalanceBot autotest: TestWheelEncoder

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TestWheelEncoder(TestGroup):
    @testcase(bucket="BalanceBot")
    def TestWheelEncoder(self):
        '''make sure wheel encoders are generally working'''
        ex = None
        try:
            self.set_parameter("WENC_TYPE", 10)
            self.set_parameter("AHRS_EKF_TYPE", 10)
            self.reboot_sitl()
            self.set_parameter("WENC2_TYPE", 10)
            self.set_parameter("WENC_POS_Y", 0.075)
            self.set_parameter("WENC2_POS_Y", -0.075)
            self.reboot_sitl()
            self.change_mode("HOLD")
            self.wait_ready_to_arm()
            self.change_mode("ACRO")
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
        except Exception as e:  # noqa: BLE001
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            self.disarm_vehicle()
            ex = e
        self.reboot_sitl()
        if ex is not None:
            raise ex
