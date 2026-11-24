'''
Rover autotest: SET_POSITION_TARGET_LOCAL_NED

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class SET_POSITION_TARGET_LOCAL_NED(TestGroup):
    @testcase(bucket="Rover")
    def SET_POSITION_TARGET_LOCAL_NED(self, target_sysid=None, target_compid=1):
        '''Test handling of SET_POSITION_TARGET_LOCAL_NED'''
        if target_sysid is None:
            target_sysid = self.sysid_thismav()
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        ofs_x = 30.0
        ofs_y = 30.0

        def send_target():
            self.mav.mav.set_position_target_local_ned_send(
                0, # time_boot_ms
                target_sysid,
                target_compid,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE,
                ofs_x,  # pos-x
                ofs_y,  # pos-y
                0,     # pos-z
                0,     # vel-x
                0,     # vel-y
                0,     # vel-z
                0,     # acc-x
                0,     # acc-y
                0,     # acc-z
                0,     # yaw
                0,     # yaw rate
            )

        self.wait_distance_to_local_position(
            (ofs_x, ofs_y, 0),
            distance_min=0,
            distance_max=3,
            timeout=60,
            called_function=lambda last_value, target : send_target(),
            minimum_duration=5,  # make sure we stop!
        )

        self.do_RTL()
        self.disarm_vehicle()
