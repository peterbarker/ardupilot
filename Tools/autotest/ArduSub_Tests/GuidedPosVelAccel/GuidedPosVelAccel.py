'''
Sub autotest: GuidedPosVelAccel

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class GuidedPosVelAccel(TestGroup):
    @testcase(bucket="Sub")
    def GuidedPosVelAccel(self):
        """Test Guided_PosVelAccel mode via SET_POSITION_TARGET_LOCAL_NED"""

        pva_mode = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

        speed = 0.5

        self.set_parameter('WP_SPD', speed)

        self.dive(-15)

        # Run back and forth between 2 locations
        distance = 30
        timeout = distance / speed + 20  # Add time to accelerate and decelerate

        runs = [{
            'bearing': 180,
            'target_alt': -15,
        }, {
            'bearing': 0,
            'target_alt': -15,
        }]

        # Stay in GUIDED mode for the duration
        self.change_mode('GUIDED')

        for run in runs:
            msg = self.assert_receive_message('LOCAL_POSITION_NED')
            start_x = msg.x
            start_y = msg.y

            dx = distance * math.cos(math.radians(run['bearing']))
            dy = distance * math.sin(math.radians(run['bearing']))
            dest_x = start_x + dx
            dest_y = start_y + dy
            dest_z = -run['target_alt']

            accel_mag = 0.2
            acc_x = accel_mag * math.cos(math.radians(run['bearing']))
            acc_y = accel_mag * math.sin(math.radians(run['bearing']))
            acc_z = 0.0

            self.progress(f"Sending target: x={dest_x:.2f}, y={dest_y:.2f}, z={dest_z:.2f} (bearing={run['bearing']})")

            self.mav.mav.set_position_target_local_ned_send(
                0,  # timestamp
                1, 1,  # target system/component IDs
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                pva_mode,
                dest_x, dest_y, dest_z,  # position target
                0, 0, 0,  # velocity target
                acc_x, acc_y, acc_z,  # acceleration target
                0, 0  # yaw, yawrate
            )

            self.wait_distance_to_local_position((dest_x, dest_y, dest_z), 0, 0.5, timeout=timeout)

        self.disarm_vehicle()
