'''
Sub autotest: ModeChanges

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase

from ardusub import Joystick


class ModeChanges(TestGroup):
    @testcase(bucket="Sub")
    def ModeChanges(self, delta=0.2):
        """Check if alternating between ALTHOLD, STABILIZE, POSHOLD and SURFTRAK (mode 21) affects altitude"""
        self.wait_ready_to_arm()
        self.arm_vehicle()
        # zero buoyancy and no baro noise
        self.set_parameter("SIM_BUOYANCY", 0)
        self.set_parameter("SIM_BARO_RND", 0)
        # dive a bit to make sure we are not surfaced
        self.change_mode('STABILIZE')
        self.set_rc(Joystick.Throttle, 1350)
        self.delay_sim_time(10, reason="vehicle to dive")
        self.set_rc(Joystick.Throttle, 1500)
        self.delay_sim_time(3, reason="vehicle to settle after dive")
        # start the test itself, go through some modes and check if anything changes
        previous_altitude = self.assert_receive_message('VFR_HUD').alt
        self.change_mode('ALT_HOLD')
        self.delay_sim_time(2, reason="ALT_HOLD to take effect")
        self.change_mode('POSHOLD')
        self.delay_sim_time(2, reason="POSHOLD to take effect")
        self.change_mode('STABILIZE')
        self.delay_sim_time(2, reason="STABILIZE to take effect")
        self.change_mode(21)
        self.delay_sim_time(2, reason="SURFTRAK to take effect")
        self.change_mode('ALT_HOLD')
        self.delay_sim_time(2, reason="ALT_HOLD to take effect")
        self.change_mode('STABILIZE')
        self.delay_sim_time(2, reason="STABILIZE to take effect")
        self.change_mode('ALT_HOLD')
        self.delay_sim_time(2, reason="ALT_HOLD to take effect")
        self.change_mode(21)
        self.delay_sim_time(2, reason="SURFTRAK to take effect")
        self.change_mode('MANUAL')
        self.disarm_vehicle()
        final_altitude = self.assert_receive_message('VFR_HUD').alt
        if abs(previous_altitude - final_altitude) > delta:
            raise NotAchievedException(
                f"Changing modes affected depth with no throttle input!, "
                f"started at {previous_altitude}, ended at {final_altitude}"
            )
