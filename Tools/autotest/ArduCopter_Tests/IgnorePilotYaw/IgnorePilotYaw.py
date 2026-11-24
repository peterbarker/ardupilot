'''
Copter autotest: IgnorePilotYaw

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class IgnorePilotYaw(TestGroup):
    @testcase(bucket="CopterTests2a")
    def IgnorePilotYaw(self):
        '''test RTL_OPTIONS bit 2, "IgnorePilotYaw"'''
        self.takeoff(10, mode='GUIDED')
        self.fly_guided_move_local(10, 10, 10, timeout=100)

        orig_yaw = self.assert_receive_message('VFR_HUD').heading

        def ensure_heading_stays_constant(mav, m):
            print(m.get_type())
            if m.get_type() != "VFR_HUD":
                return
            new_yaw = m.heading
            if abs(orig_yaw - new_yaw) > 15:
                raise NotAchievedException(f"Vehicle yawed! ({orig_yaw=} {new_yaw=}")

        self.context_push()
        self.install_message_hook_context(ensure_heading_stays_constant)

        self.change_mode('RTL')

        self.set_parameter('RTL_OPTIONS', 1 << 2)
        self.set_rc(4, 2000)
        self.delay_sim_time(5, reason="RTL to progress with yaw override")
        self.context_pop()

        self.wait_disarmed()
