'''
Rover autotest: MAV_CMD_NAV_RETURN_TO_LAUNCH

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_RETURN_TO_LAUNCH(TestGroup):
    @testcase(bucket="Rover")
    def MAV_CMD_NAV_RETURN_TO_LAUNCH(self):
        '''test MAV_CMD_NAV_RETURN_TO_LAUNCH mavlink command'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        here = self.get_location()
        target_loc = self.offset_location_ne(here, 2000, 0)
        self.send_guided_mission_item(target_loc)
        # the vehicle starts pointing away from the target, so it turns
        # before it makes any ground: wait_distance_to_home()'s default
        # ten seconds is enough only if that turn is quick, and one run
        # spent them reaching 14.4m of the wanted 20m
        self.wait_distance_to_home(20, 100, timeout=60)

        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
        self.wait_mode('RTL')

        self.change_mode('GUIDED')

        self.run_cmd_int(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
        self.wait_mode('RTL')

        self.wait_distance_to_home(0, 5, timeout=30)
        self.disarm_vehicle()
