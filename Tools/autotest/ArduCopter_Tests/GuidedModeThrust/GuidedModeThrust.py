'''
Copter autotest: GuidedModeThrust

AP_FLAKE8_CLEAN
'''

from pymavlink import mavextra

from test_discovery import TestGroup
from test_discovery import testcase


class GuidedModeThrust(TestGroup):
    @testcase(bucket="CopterTests2b")
    def GuidedModeThrust(self):
        '''test handling of option-bit-3, where mavlink commands are
        intrepreted as thrust not climb rate'''
        self.set_parameter('GUID_OPTIONS', 8)
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.mav.mav.set_attitude_target_send(
            0, # time_boot_ms
            1, # target sysid
            1, # target compid
            0, # bitmask of things to ignore
            mavextra.euler_to_quat([0, 0, 0]), # att
            0, # roll rate  (rad/s)
            0, # pitch rate (rad/s)
            0, # yaw rate   (rad/s)
            0.5
        ) # thrust, 0 to 1
        self.wait_altitude(0.5, 100, relative=True, timeout=10)
        self.do_RTL()
