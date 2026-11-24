'''
Sub autotest: SET_POSITION_TARGET_GLOBAL_INT

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SET_POSITION_TARGET_GLOBAL_INT(TestGroup):
    @testcase(bucket="Sub")
    def SET_POSITION_TARGET_GLOBAL_INT(self):
        '''Move vehicle using SET_POSITION_TARGET_GLOBAL_INT'''
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        startpos = self.assert_receive_message('GLOBAL_POSITION_INT')

        lat = 5
        lon = 5
        alt = -10

        # send a position-control command
        self.mav.mav.set_position_target_global_int_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b1111111111111000, # mask specifying use-only-lat-lon-alt
            lat, # lat
            lon, # lon
            alt, # alt
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 200:
                raise NotAchievedException("Did not move far enough")
            pos = self.assert_receive_message('GLOBAL_POSITION_INT')
            delta = self.get_distance_int(startpos, pos)
            self.progress(f"delta={delta} (want >10)")
            if delta > 10:
                break
        self.change_mode('MANUAL')
        self.disarm_vehicle()
