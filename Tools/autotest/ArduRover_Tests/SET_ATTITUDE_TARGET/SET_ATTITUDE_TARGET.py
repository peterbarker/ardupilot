'''
Rover autotest: SET_ATTITUDE_TARGET

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra
from pymavlink import mavutil

from vehicle_test_suite import AutoTestTimeoutException

from test_discovery import TestGroup
from test_discovery import testcase


class SET_ATTITUDE_TARGET(TestGroup):
    @testcase(bucket="Rover")
    def SET_ATTITUDE_TARGET(self, target_sysid=None, target_compid=1):
        '''Test handling of SET_ATTITUDE_TARGET'''
        if target_sysid is None:
            target_sysid = self.sysid_thismav()
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 10:
                raise AutoTestTimeoutException("Didn't get to speed")
            self.mav.mav.set_attitude_target_send(
                0, # time_boot_ms
                target_sysid,
                target_compid,
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE |
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE |
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,
                mavextra.euler_to_quat([0,
                                        math.radians(0),
                                        math.radians(0)]), # att
                0, # yaw rate (rad/s)
                0, # pitch rate
                0, # yaw rate
                1) # thrust

            msg = self.assert_receive_message('VFR_HUD')
            if msg.groundspeed > 5:
                break
        self.disarm_vehicle()
