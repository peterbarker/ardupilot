'''
Copter autotest: ModeAllowsEntryWhenNoPilotInput

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class ModeAllowsEntryWhenNoPilotInput(TestGroup):
    @testcase(bucket="CopterTests2a")
    def ModeAllowsEntryWhenNoPilotInput(self):
        '''test we can't enter modes when no RC input'''
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.set_parameters({
            "AUTO_OPTIONS": 3,
            "FS_THR_ENABLE": 4,  # continue in auto
        })
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_current_waypoint(2)
        self.context_collect('STATUSTEXT')
        self.set_rc(3, 200)
        self.wait_statustext('Radio Failsafe', check_context=True)
        self.send_cmd_do_set_mode('STABILIZE')
        self.wait_statustext('in RC failsafe', check_context=True)
        self.hover()
        self.wait_statustext('Radio Failsafe Cleared', check_context=True)
        self.do_RTL()
