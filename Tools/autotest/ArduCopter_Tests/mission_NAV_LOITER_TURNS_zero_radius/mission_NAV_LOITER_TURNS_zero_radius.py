'''
Copter autotest: mission_NAV_LOITER_TURNS_zero_radius

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class mission_NAV_LOITER_TURNS_zero_radius(TestGroup):
    @testcase(bucket="CopterTests1c")
    def mission_NAV_LOITER_TURNS_zero_radius(self):
        '''radius-0 LOITER_TURNS holds position and spins yaw at CIRCLE_RATE (panorama)'''
        turns = 2
        rate_degs = 20.0
        expected_duration_s = turns * 360.0 / rate_degs
        # a zero radius cannot encode a direction in the mission item (loiter_ccw comes from
        # param3 < 0), so CIRCLE_RATE's sign is the panorama's only direction control.  A
        # negative rate must therefore spin counter-clockwise.
        self.set_parameters({
            'AUTO_OPTIONS': 3,
            'CIRCLE_RATE': -rate_degs,
        })
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                p1=turns,
                p3=0,  # zero radius: panorama
                z=20,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # the panorama timer starts when circle_start announces the command
        self.wait_statustext("Mission: circling", timeout=120)
        tstart = self.get_sim_time()
        start_pos = self.assert_receive_message('GLOBAL_POSITION_INT', timeout=5)

        # accumulate heading change and position drift until the mission moves on to RTL
        total_angle_deg = 0.0
        last_yaw_deg = None
        max_drift_m = 0.0
        while True:
            if self.get_sim_time_cached() - tstart > expected_duration_s * 2 + 30:
                raise AutoTestTimeoutException("panorama did not complete")
            m = self.assert_receive_message(
                ['ATTITUDE', 'GLOBAL_POSITION_INT', 'MISSION_CURRENT'],
                timeout=5,
            )
            m_type = m.get_type()
            if m_type == 'ATTITUDE':
                yaw_deg = math.degrees(m.yaw)
                if last_yaw_deg is not None:
                    delta_deg = yaw_deg - last_yaw_deg
                    if delta_deg > 180:
                        delta_deg -= 360
                    elif delta_deg < -180:
                        delta_deg += 360
                    total_angle_deg += delta_deg
                last_yaw_deg = yaw_deg
            elif m_type == 'GLOBAL_POSITION_INT':
                max_drift_m = max(max_drift_m, self.get_distance_int(start_pos, m))
            elif m_type == 'MISSION_CURRENT':
                if m.seq >= 3:
                    break
        elapsed_s = self.get_sim_time_cached() - tstart

        if elapsed_s < expected_duration_s * 0.9:
            raise NotAchievedException(
                "Panorama completed too quickly (want>=%.1fs got %.1fs)" %
                (expected_duration_s * 0.9, elapsed_s))
        # CIRCLE_RATE was set negative, so the spin must be counter-clockwise through roughly
        # the commanded total angle
        target_angle_deg = -turns * 360.0
        if abs(total_angle_deg - target_angle_deg) > 80:
            raise NotAchievedException(
                "Panorama spun %.1fdeg, expected %.1fdeg (negative CIRCLE_RATE = CCW)" %
                (total_angle_deg, target_angle_deg))
        if max_drift_m > 5:
            raise NotAchievedException(
                "Vehicle did not hold position during panorama (drift %.1fm)" % max_drift_m)

        self.wait_disarmed()
