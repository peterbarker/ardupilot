'''
Rover autotest: MAV_CMD_DO_CHANGE_SPEED

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_CHANGE_SPEED(TestGroup):
    @testcase(bucket="Rover")
    def MAV_CMD_DO_CHANGE_SPEED(self):
        '''test MAV_CMD_NAV_RETURN_TO_LAUNCH mavlink command'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        original_loc = self.get_location()
        here = original_loc
        target_loc = self.offset_location_ne(here, 2000, 0)
        self.send_guided_mission_item(target_loc)
        self.wait_distance_to_home(20, 100)

        speeds = 3, 7, 12, 4

        for speed in speeds:
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, p2=speed)
            self.wait_groundspeed(speed-0.5, speed+0.5, minimum_duration=5)

        self.send_guided_mission_item(original_loc)

        for speed in speeds:
            self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, p2=speed)
            self.wait_groundspeed(speed-0.5, speed+0.5, minimum_duration=5)

        self.change_mode('RTL')

        self.wait_distance_to_home(0, 5, timeout=30)
        self.disarm_vehicle()
