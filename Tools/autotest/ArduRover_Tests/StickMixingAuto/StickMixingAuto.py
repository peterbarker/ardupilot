'''
Rover autotest: StickMixingAuto

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class StickMixingAuto(TestGroup):
    @testcase(bucket="Rover")
    def StickMixingAuto(self):
        '''Ensure Stick Mixing works in auto'''
        items = []
        self.set_parameter('STICK_MIXING', 1)
        # home
        items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0),)
        # 1 waypoint a long way away
        items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 0),)
        self.upload_simple_relhome_mission(items)
        if self.mavproxy is not None:
            # handy for getting pretty pictures
            self.mavproxy.send("wp list\n")
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(1, 1150)
        self.wait_heading(45)
        self.wait_heading(90)
        self.disarm_vehicle()
