'''
Copter autotest: ModeFollow_with_FOLLOW_TARGET

AP_FLAKE8_CLEAN
'''

import copy
import math

from pymavlink import quaternion
from pymavlink.rotmat import Vector3

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class ModeFollow_with_FOLLOW_TARGET(TestGroup):
    @testcase(bucket="CopterTests1d")
    def ModeFollow_with_FOLLOW_TARGET(self):
        '''test ModeFollow passing in FOLLOW_TARGET messages'''
        foll_ofs_x = 30 # metres
        self.set_parameters({
            "FOLL_ENABLE": 1,
            "FOLL_SYSID": self.mav.source_system,
            "FOLL_OFS_X": -foll_ofs_x,
            "FOLL_OFS_TYPE": 1, # relative to other vehicle heading
        })
        self.takeoff(10, mode="LOITER")
        self.context_push()
        self.context_set_speedup(1)
        self.change_mode("FOLLOW")
        new_loc = self.get_location()
        new_loc_offset_n = 40
        new_loc_offset_e = 60
        self.location_offset_ne(new_loc, new_loc_offset_n, new_loc_offset_e)
        self.progress("new_loc: %s" % str(new_loc))
        heading = 0
        if self.mavproxy is not None:
            self.mavproxy.send("map icon %f %f greenplane %f\n" %
                               (new_loc.lat, new_loc.lng, heading))

        expected_loc = copy.copy(new_loc)
        self.location_offset_ne(expected_loc, -foll_ofs_x, 0)
        if self.mavproxy is not None:
            self.mavproxy.send("map icon %f %f hoop\n" %
                               (expected_loc.lat, expected_loc.lng))
        self.progress("expected_loc: %s" % str(expected_loc))

        def send_target():
            vel_n = 3  # m/s
            vel_e = 2
            # vel_d = -1
            heading = math.atan2(vel_n, vel_e)
            attitude = quaternion.Quaternion([
                math.radians(0),
                math.radians(0),
                math.radians(heading)
            ])
            now = self.get_sim_time_cached()
            self.mav.mav.follow_target_send(
                int(now * 1000), # time_boot_ms
                1 << 0 | 1 << 1 | 1 << 3,  # pos, vel, att+rates
                int(new_loc.lat * 1e7),
                int(new_loc.lng * 1e7),
                new_loc.get_alt_m(AltFrame.ABSOLUTE), # alt in m
                [0, 0, 0],  # velocity m/s
                [0, 0, 0],  # acceleration m/s/s
                attitude,   # attitude quaternion
                [0, 0, 0],  # rates
                [0, 0, 0],  # covariances
                0  # custom state
            )
        self.wait_location(
            expected_loc,
            fn=send_target,
            fn_interval=0.5,
            accuracy=3,
            timeout=60,
        )

        self.start_subtest("Trying follow-with-velocity mode")
        self.change_mode('LOITER')
        self.set_parameter('FOLL_ALT_TYPE', 1)  # use relative-home data
        new_loc = self.get_location()
        self.change_mode('FOLLOW')
        last_loop_s = self.get_sim_time_cached()

        vel_n = 3  # m/s
        vel_e = 2
        vel_d = 0

        mul = 40
        self.location_offset_ne(expected_loc, mul*vel_n, mul*vel_e)

        def send_target_vel():
            nonlocal last_loop_s

            now = self.get_sim_time_cached()
            dt = now - last_loop_s
            last_loop_s = now

            new_loc_offset_n = vel_n * dt
            new_loc_offset_e = vel_e * dt
            new_loc_offset_u = vel_d * dt * -1
            self.location_offset_ne(new_loc, new_loc_offset_n, new_loc_offset_e)
            new_loc.offset_up_m(new_loc_offset_u)

            # update heading
            heading = math.atan2(vel_n, vel_e)
            attitude = quaternion.Quaternion([
                math.radians(0),
                math.radians(0),
                math.radians(heading)
            ])
            now = self.get_sim_time_cached()
            self.mav.mav.follow_target_send(
                int(now * 1000), # time_boot_ms
                1 << 0 | 1 << 1 | 1 << 3,  # pos, vel, att+rates
                int(new_loc.lat * 1e7),
                int(new_loc.lng * 1e7),
                new_loc.get_alt_m(AltFrame.ABSOLUTE), # alt in m
                [vel_n, vel_e, vel_d],  # velocity m/s
                [0, 0, 0],  # acceleration m/s/s
                attitude,   # attitude quaternion
                [0, 0, 0],  # rates
                [0, 0, 0],  # covariances
                0  # custom state
            )
        self.wait_speed_vector(
            Vector3(vel_n, vel_e, 0),
            timeout=100,
            called_function=lambda plop, empty: send_target_vel(),
            minimum_duration=2,
        )

        self.context_pop()
        self.do_RTL()
