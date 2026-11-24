'''
Copter autotest: FlyMissionTwiceWithReset

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class FlyMissionTwiceWithReset(TestGroup):
    @testcase(bucket="CopterTests2b")
    def FlyMissionTwiceWithReset(self):
        '''Fly a mission twice in a row without changing modes in between.
        Allow the mission to complete, then reset the mission state machine and restart the mission.'''

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
            # Use the "Reset Mission" param of DO_SET_MISSION_CURRENT to reset mission state machine
            self.set_current_waypoint_using_mav_cmd_do_set_mission_current(seq=0, reset=1)
            self.arm_vehicle()
            self.wait_waypoint(num_wp-1, num_wp-1)
            self.wait_disarmed()
            self.delay_sim_time(20, reason="SITL to reset before next run")
