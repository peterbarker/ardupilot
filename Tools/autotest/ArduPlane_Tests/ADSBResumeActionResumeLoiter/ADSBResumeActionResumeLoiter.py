'''
Plane autotest: ADSBResumeActionResumeLoiter

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class ADSBResumeActionResumeLoiter(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def ADSBResumeActionResumeLoiter(self):
        '''ensure we resume auto mission or enter loiter'''
        self.set_parameters({
            "ADSB_TYPE": 1,
            "AVD_ENABLE": 1,
            "AVD_F_ACTION": mavutil.mavlink.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY,
            "AVD_F_RCVRY": 3,  # resume auto or loiter
        })
        self.reboot_sitl()
        self.takeoff(50)
        # fly North, create thread to east, wait for flying east
        self.start_subtest("Testing loiter resume")
        self.reach_heading_manual(0)
        here = self.get_location()
        self.test_adsb_send_threatening_adsb_message(here, offset_ne=(0, 30))
        self.wait_mode('AVOID_ADSB')
        # recovery has the vehicle circling a point... but we don't
        # know which point.  So wait 'til it looks like it is
        # circling, then grab the point, then check we're circling
        # it...
        self.wait_heading(290)
        self.wait_heading(300)
        dest = self.position_target_loc()
        REALLY_BAD_FUDGE_FACTOR = 1.25  # FIXME
        expected_radius = REALLY_BAD_FUDGE_FACTOR * self.get_parameter('WP_LOITER_RAD')
        self.wait_circling_point_with_radius(dest, expected_radius)

        self.start_subtest("Testing mission resume")
        self.reach_heading_manual(270)
        self.load_generic_mission("CMAC-circuit.txt", strict=False)
        self.change_mode('AUTO')
        self.wait_current_waypoint(2)
        self.test_adsb_send_threatening_adsb_message(here, offset_ne=(0, 30))
        self.wait_mode('AVOID_ADSB')
        self.wait_mode('AUTO')

        self.fly_home_land_and_disarm()
