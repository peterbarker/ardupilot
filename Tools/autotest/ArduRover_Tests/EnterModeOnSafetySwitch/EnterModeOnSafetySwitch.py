'''
Rover autotest: EnterModeOnSafetySwitch

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class EnterModeOnSafetySwitch(TestGroup):
    @testcase(bucket="Rover")
    def EnterModeOnSafetySwitch(self):
        '''test mode enter behaviour when there's a safety switch involved'''
        self.set_parameters({
            "SIM_GPS1_ENABLE": 0,
        })
        self.reboot_sitl()
        self.wait_mode('MANUAL')
        self.wait_prearm_sys_status_healthy()
        self.arm_vehicle()
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=self.get_mode_from_mode_mapping('GUIDED'),
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )
        self.set_safetyswitch_on()
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=self.get_mode_from_mode_mapping('GUIDED'),
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )
        self.set_safetyswitch_off()
        self.disarm_vehicle()
