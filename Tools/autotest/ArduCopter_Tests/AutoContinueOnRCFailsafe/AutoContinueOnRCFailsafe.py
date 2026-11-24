'''
Copter autotest: AutoContinueOnRCFailsafe

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class AutoContinueOnRCFailsafe(TestGroup):
    @testcase(bucket="CopterTests2d")
    def AutoContinueOnRCFailsafe(self):
        '''check LOITER when entered after RC failsafe is ignored in auto'''
        self.set_parameters({
            "FS_OPTIONS": 1,  # 1 is "RC continue if in auto"
        })

        self.upload_simple_relhome_mission([
            #                                      N   E  U
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,   0, 0, 10),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 20, 0, 10),
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, p1=1),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 40, 0, 10),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 120, 0, 10),
        ])

        self.takeoff(mode='LOITER')
        self.set_rc(1, 1200)
        self.delay_sim_time(1, reason="pilot desired input to build up")  # build up some pilot desired stuff
        self.change_mode('AUTO')
        self.wait_waypoint(2, 2, max_dist_to_final_wp_m=3)
        self.set_parameters({
            'SIM_RC_FAIL': 1,
        })
#        self.set_rc(1, 1500)  # note we are still in RC fail!
        self.wait_waypoint(3, 3, max_dist_to_final_wp_m=3)
        self.assert_mode_is('AUTO')
        self.change_mode('LOITER')
        self.wait_groundspeed(0, 0.1, minimum_duration=30, timeout=450)
        # do_RTL plays with RC inputs, don't use it.
        self.change_mode("RTL")
        self.wait_rtl_complete()
