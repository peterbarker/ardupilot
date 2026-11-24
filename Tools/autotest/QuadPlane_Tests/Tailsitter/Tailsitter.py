'''
QuadPlane autotest: Tailsitter

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class Tailsitter(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def Tailsitter(self):
        '''tailsitter test'''
        self.set_parameter('Q_FRAME_CLASS', 10)
        self.set_parameter('Q_ENABLE', 1)
        self.set_parameter('Q_TAILSIT_ENABLE', 1)

        self.reboot_sitl()
        self.wait_ready_to_arm()
        value_before = self.get_servo_channel_value(3)
        self.progress("Before: %u" % value_before)
        self.change_mode('QHOVER')
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 60:
                break
            value_after = self.get_servo_channel_value(3)
            self.progress("After: t=%f output=%u" % ((now - tstart), value_after))
            if value_before != value_after:
                raise NotAchievedException("Changed throttle output on mode change to QHOVER")
        self.disarm_vehicle()
