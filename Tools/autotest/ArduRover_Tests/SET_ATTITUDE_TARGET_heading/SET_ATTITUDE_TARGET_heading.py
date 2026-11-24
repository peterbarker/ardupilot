'''
Rover autotest: SET_ATTITUDE_TARGET_heading

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra
from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SET_ATTITUDE_TARGET_heading(TestGroup):
    @testcase(bucket="Rover")
    def SET_ATTITUDE_TARGET_heading(self, target_sysid=None, target_compid=1):
        '''Test heading control and telemetry for SET_ATTITUDE_TARGET'''
        if target_sysid is None:
            target_sysid = self.sysid_thismav()
        self.context_push()
        self.context_set_message_rate_hz('NAV_CONTROLLER_OUTPUT', 10)
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        for angle in 0, 45, 290, 70, 180, 359, 1, 0:
            self.SET_ATTITUDE_TARGET_heading_test_target(angle, target_sysid, target_compid)

        self.start_subtest("Switch from a nonzero heading target to turn-rate control")
        self.SET_ATTITUDE_TARGET_heading_test_target(45, target_sysid, target_compid)

        def poke_turn_rate(value, target):
            self.mav.mav.set_attitude_target_send(
                0, # time_boot_ms
                target_sysid,
                target_compid,
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE |
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE |
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,
                [1, 0, 0, 0], # ignored attitude
                0, # roll rate (rad/s)
                0, # pitch rate
                0, # yaw rate
                1) # thrust

        self.drain_mav()
        poke_turn_rate(None, None)
        self.wait_guided_nav_bearing(0, poke_turn_rate)
        self.disarm_vehicle()
        self.context_pop()

    def wait_guided_nav_bearing(self, bearing, called_function):
        def get_nav_bearing():
            m = self.assert_receive_message('NAV_CONTROLLER_OUTPUT')
            if m.target_bearing != 0:
                raise NotAchievedException("Expected target_bearing=0, got %s" % m.target_bearing)
            if not -180 <= m.nav_bearing <= 180:
                raise NotAchievedException("nav_bearing outside signed degree range: %s" % m.nav_bearing)
            return m.nav_bearing

        self.wait_and_maintain(
            value_name='nav_bearing',
            target=bearing,
            current_value_getter=get_nav_bearing,
            validator=lambda value, target: self.heading_delta(value, target) <= 1,
            accuracy=1,
            timeout=10,
            minimum_duration=1,
            called_function=called_function,
        )

    def SET_ATTITUDE_TARGET_heading_test_target(self, angle, target_sysid, target_compid):
        if target_sysid is None:
            target_sysid = self.sysid_thismav()

        def poke_set_attitude(value, target):
            self.mav.mav.set_attitude_target_send(
                0, # time_boot_ms
                target_sysid,
                target_compid,
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE |
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE |
                mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE,
                mavextra.euler_to_quat([
                    math.radians(0),
                    math.radians(0),
                    math.radians(angle)
                ]), # att
                0, # roll rate (rad/s)
                0, # pitch rate
                0, # yaw rate
                1) # thrust

        self.drain_mav()
        poke_set_attitude(None, None)
        self.wait_heading(angle, called_function=poke_set_attitude, minimum_duration=5)
        bearing = angle if angle <= 180 else angle - 360
        self.wait_guided_nav_bearing(bearing, poke_set_attitude)
