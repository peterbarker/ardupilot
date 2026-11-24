'''
Rover autotest: MAV_CMD_DO_SET_REVERSE

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_SET_REVERSE(TestGroup):
    @testcase(bucket="Rover")
    def MAV_CMD_DO_SET_REVERSE(self):
        '''test MAV_CMD_DO_SET_REVERSE command'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        here = self.get_location()
        target_loc = self.offset_location_ne(here, 2000, 0)
        self.send_guided_mission_item(target_loc)

        self.wait_groundspeed(3, 100, minimum_duration=5)

        for method in self.run_cmd, self.run_cmd_int:
            self.progress("Forwards!")
            method(mavutil.mavlink.MAV_CMD_DO_SET_REVERSE, p1=0)
            self.wait_heading(0)

            self.progress("Backwards!")
            method(mavutil.mavlink.MAV_CMD_DO_SET_REVERSE, p1=1)
            self.wait_heading(180)

            self.progress("Forwards!")
            method(mavutil.mavlink.MAV_CMD_DO_SET_REVERSE, p1=0)
            self.wait_heading(0)

        self.disarm_vehicle()
