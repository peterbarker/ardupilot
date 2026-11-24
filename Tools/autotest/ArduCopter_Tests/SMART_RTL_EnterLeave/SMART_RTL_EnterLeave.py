'''
Copter autotest: SMART_RTL_EnterLeave

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class SMART_RTL_EnterLeave(TestGroup):
    @testcase(bucket="CopterTests2b", disabled="Causes a panic")
    def SMART_RTL_EnterLeave(self):
        '''check SmartRTL behaviour when entering/leaving'''
        # we had a bug where we would consume points when re-entering smartrtl

        self.upload_simple_relhome_mission([
            #                                      N   E  U
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,   0, 0, 10),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.set_parameter('AUTO_OPTIONS', 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.change_mode('ALT_HOLD')
        self.change_mode('SMART_RTL')
        self.change_mode('ALT_HOLD')
        self.change_mode('SMART_RTL')
