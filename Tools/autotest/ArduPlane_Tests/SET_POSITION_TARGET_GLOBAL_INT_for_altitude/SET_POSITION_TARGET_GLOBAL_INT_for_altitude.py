'''
Plane autotest: SET_POSITION_TARGET_GLOBAL_INT_for_altitude

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import MAV_POS_TARGET_TYPE_MASK

from test_discovery import TestGroup
from test_discovery import testcase


class SET_POSITION_TARGET_GLOBAL_INT_for_altitude(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def SET_POSITION_TARGET_GLOBAL_INT_for_altitude(self):
        '''test changing altitude using SET_POSITION_TARGET_GLOBAL_INT_for_altitude in guided mode'''
        self.takeoff(30, mode='TAKEOFF')
        self.change_mode('GUIDED')
        target_alt = 40
        self.mav.mav.set_position_target_global_int_send(
            0, # timestamp
            self.mav.target_system, # target system_id
            self.mav.target_component, # target component id
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_POS_TARGET_TYPE_MASK.ALT_ONLY,
            0, # lat
            0, # lon
            target_alt, # alt
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )
        self.wait_altitude(
            target_alt-1,
            target_alt+1,
            minimum_duration=10,
            timeout=120,
            relative=True,
        )

        self.progress("Ensure ignore bit is honoured")
        self.mav.mav.set_position_target_global_int_send(
            0, # timestamp
            self.mav.target_system, # target system_id
            self.mav.target_component, # target component id
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_POS_TARGET_TYPE_MASK.IGNORE_ALL, # mask specifying use-only-alt
            0, # lat
            0, # lon
            target_alt, # alt
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )
        self.wait_altitude(
            target_alt-1,
            target_alt+1,
            timeout=60,
            minimum_duration=10,
            relative=True,
        )
        self.fly_home_land_and_disarm()
