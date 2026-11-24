'''
Copter autotest: RTLStoppingDistanceSpeed

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RTLStoppingDistanceSpeed(TestGroup):
    @testcase(bucket="CopterTests2b", disabled="Currently fails due to vehicle going off-course")
    def RTLStoppingDistanceSpeed(self):
        '''test stopping distance unaffected by RTL speed'''
        self.upload_simple_relhome_mission([
            #                                              N  E   U
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,          0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,       200, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0,  0),
        ])
        self.set_parameters({
            "AUTO_OPTIONS": 3,
        })
        self.context_push()
        self.set_parameters({
            'RTL_SPEED_MS': 1,  # m/s
        })

        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_current_waypoint(2)

        self.progress("Waiting for vehicle to get up to speed")
        wp_speed = self.get_parameter('WP_SPD')
        self.wait_groundspeed(wp_speed-0.1, wp_speed+0.1)

        rtl_start_pos = self.get_local_position_NED()

        # we accelerate so hard we can miss the zero value!
        self.context_set_message_rate_hz('VFR_HUD', 50)

        # note that there's going to be a significant latency involved
        # between taking rtl_start_pos and entering RTL, but we're
        # really interested in deviation to the east/west, and we are
        # travelling due North, ...
        self.change_mode('RTL')

        self.progress("Waiting for vehicle to stop")
        self.wait_groundspeed(-0.3, 0.3)
        rtl_stopping_point_pos = self.get_local_position_NED()

        self.progress("Checking vehicle deviation from track")
        y_delta = abs(rtl_start_pos.y - rtl_stopping_point_pos.y)
        self.progress(f"deviated {y_delta}m from track")
        if y_delta > 0.2:
            raise NotAchievedException(f"RTL deviated from track {y_delta}m")
        self.context_pop()
        self.change_mode('LOITER')
        self.do_RTL()
