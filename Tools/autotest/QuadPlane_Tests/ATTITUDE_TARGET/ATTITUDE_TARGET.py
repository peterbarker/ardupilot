'''
QuadPlane autotest: ATTITUDE_TARGET

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil
from pymavlink import quaternion

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ATTITUDE_TARGET(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def ATTITUDE_TARGET(self):
        '''check ATTITUDE_TARGET reports the VTOL attitude controller's target'''
        self.context_set_message_rate_hz('ATTITUDE_TARGET', 10)
        self.context_set_message_rate_hz('EXTENDED_SYS_STATE', 10)

        self.start_subtest("not sent when the VTOL attitude controller is not running")
        self.change_mode('FBWA')
        self.assert_not_receive_message('ATTITUDE_TARGET', timeout=2)

        self.takeoff(20, mode='QHOVER')

        self.start_subtest("target follows pilot roll input")
        self.set_rc(1, 1700)
        self.delay_sim_time(5, reason="vehicle to reach commanded roll")
        m = self.assert_receive_message('ATTITUDE_TARGET', verbose=True)
        attitude = self.assert_receive_message('ATTITUDE', verbose=True)
        self.set_rc(1, 1500)
        if m.type_mask != 0:
            raise NotAchievedException(f"Want type_mask=0 got {m.type_mask}")
        if not 0 <= m.thrust <= 1:
            raise NotAchievedException(f"thrust {m.thrust} out of range")
        (target_roll, target_pitch, target_yaw) = quaternion.Quaternion(m.q).euler
        if math.degrees(target_roll) < 10:
            raise NotAchievedException(f"Expected a positive roll target got {math.degrees(target_roll)}")
        if abs(math.degrees(target_roll - attitude.roll)) > 5:
            raise NotAchievedException(
                f"Roll target {math.degrees(target_roll)} far from roll {math.degrees(attitude.roll)}")

        self.start_subtest("target yaw leads the vehicle's yaw")
        # slow the yaw response so the yaw target runs well ahead of the vehicle:
        yaw_gains = self.get_parameters(["Q_A_RAT_YAW_P", "Q_A_RAT_YAW_I"])
        self.set_parameters({name: value * 0.1 for name, value in yaw_gains.items()})
        self.set_rc(4, 2000)
        self.delay_sim_time(2, reason="yaw target to run ahead")
        frame = self.get_messages_frame(['ATTITUDE_TARGET', 'ATTITUDE'], timeout=10)
        self.set_rc(4, 1500)
        target_yaw = math.degrees(quaternion.Quaternion(frame['ATTITUDE_TARGET'].q).euler[2])
        yaw = math.degrees(frame['ATTITUDE'].yaw)
        lead = (target_yaw - yaw + 180) % 360 - 180
        self.progress(f"yaw target {target_yaw:.1f} yaw {yaw:.1f} lead {lead:.1f}")
        if lead < 10:
            raise NotAchievedException(f"Yaw target is not ahead of the vehicle's yaw (lead={lead:.1f})")
        self.set_parameters(yaw_gains)
        self.delay_sim_time(5, reason="yaw to settle")

        self.start_subtest("not sent in fixed-wing flight")
        self.change_mode('FBWA')
        self.set_rc(3, 1900)
        self.wait_extended_sys_state(
            mavutil.mavlink.MAV_VTOL_STATE_FW,
            mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
            timeout=60,
        )
        self.delay_sim_time(2, reason="messages sent during the transition to arrive")
        self.drain_mav()
        self.assert_not_receive_message('ATTITUDE_TARGET', timeout=2)
        self.set_rc(3, 1500)

        self.change_mode('QLAND')
        self.wait_disarmed(timeout=120)
