'''
Copter autotest: FlyMissionTwice

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class FlyMissionTwice(TestGroup):
    @testcase(bucket="CopterTests2b", disabled="See https://github.com/ArduPilot/ardupilot/pull/18561")
    def FlyMissionTwice(self):
        '''fly a mission twice in a row without changing modes in between.
        Seeks to show bugs in mission state machine'''

        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])

        num_wp = self.get_mission_count()
        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        for i in 1, 2:
            self.progress("run %u" % i)
            self.arm_vehicle()
            self.wait_waypoint(num_wp-1, num_wp-1)
            self.wait_disarmed()
            self.delay_sim_time(20, reason="SITL to reset before next run")
