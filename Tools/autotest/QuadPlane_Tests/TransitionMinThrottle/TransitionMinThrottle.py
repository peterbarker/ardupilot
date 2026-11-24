'''
QuadPlane autotest: TransitionMinThrottle

AP_FLAKE8_CLEAN
'''

import operator

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class TransitionMinThrottle(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def TransitionMinThrottle(self):
        '''Ensure that TKOFF_THR_MIN is applied during the forward transition'''
        wps = self.create_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.check_mission_upload_download(wps)
        self.set_parameter('TKOFF_THR_MIN', 80)

        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.wait_current_waypoint(2)
        # Wait for 5 seconds into the transition.
        self.delay_sim_time(5, reason="transition to progress")
        # Ensure TKOFF_THR_MIN is still respected.
        thr_min = self.get_parameter('TKOFF_THR_MIN')
        self.wait_servo_channel_value(3, 1000+thr_min*10, comparator=operator.eq)

        self.fly_home_land_and_disarm()
