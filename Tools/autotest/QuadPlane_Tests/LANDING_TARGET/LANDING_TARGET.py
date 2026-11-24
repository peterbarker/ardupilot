'''
QuadPlane autotest: LANDING_TARGET

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class LANDING_TARGET(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def LANDING_TARGET(self):
        '''VTOL precision landing on a target reported by LANDING_TARGET'''
        self.install_applet_script_context("plane_precland.lua")
        self.set_parameters({
            "SCR_ENABLE": 1,
            "PLND_ENABLED": 1,
            "PLND_TYPE": 1,  # MAVLink
        })
        self.reboot_sitl()
        self.context_collect('STATUSTEXT')
        self.scripting_restart()
        self.wait_text("PLND: Loaded", check_context=True)
        self.wait_ready_to_arm()

        def send_landing_target(frame, angle_x, angle_y, distance):
            self.mav.mav.landing_target_send(
                0, # time_usec
                1, # target_num
                frame,
                angle_x,
                angle_y,
                distance,
                0.01, # size_x
                0.01, # size_y
            )

        self.start_subtest("frames other than BODY_FRD and LOCAL_FRD are refused")
        send_landing_target(mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0, 0, 10)
        self.wait_statustext("Plnd: Frame not supported", check_context=True)

        takeoff_loc = self.get_location()
        target = self.offset_location_ne(takeoff_loc, 20, 0)

        def report_target(mav, m):
            if m.get_type() != 'GLOBAL_POSITION_INT':
                return
            attitude = self.mav.messages.get('ATTITUDE')
            if attitude is None:
                return
            down = m.relative_alt * 0.001
            if down < 1:
                return
            here = Location.latlon_only(m.lat * 1e-7, m.lon * 1e-7)
            distance_ne = self.get_distance(here, target)
            bearing = math.radians(self.get_bearing(here, target))
            north = distance_ne * math.cos(bearing)
            east = distance_ne * math.sin(bearing)
            # rotate into a yaw-aligned forward-right-down frame:
            yaw = attitude.yaw
            forward = north * math.cos(yaw) + east * math.sin(yaw)
            right = -north * math.sin(yaw) + east * math.cos(yaw)
            send_landing_target(
                mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
                math.atan2(right, down),
                -math.atan2(forward, down),
                math.sqrt(forward*forward + right*right + down*down),
            )

        self.context_set_message_rate_hz('GLOBAL_POSITION_INT', 10)
        self.change_mode("GUIDED")
        self.arm_vehicle()
        self.user_takeoff(alt_min=30)
        self.install_message_hook_context(report_target)
        self.wait_text("PrecLand: Target Found", check_context=True, timeout=60)

        # the applet only uses the target while landing:
        self.change_mode("QLAND")
        self.wait_text("PLND: Target Acquired", check_context=True, timeout=60)
        self.wait_disarmed(timeout=180)
        landing_loc = self.get_location()
        error = self.get_distance(target, landing_loc)
        moved = self.get_distance(takeoff_loc, landing_loc)
        self.progress(f"Target error {error:.1f}m, landed {moved:.1f}m from takeoff")
        if error > 2:
            raise NotAchievedException(f"Landed {error:.1f}m from target")
