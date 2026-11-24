'''
Rover autotest: PolyFenceAvoidance

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PolyFenceAvoidance(TestGroup):
    def drive_somewhere_stop_at_boundary(self,
                                         loc,
                                         expected_stopping_point,
                                         expected_distance_epsilon=1.0,
                                         target_system=1,
                                         target_component=1,
                                         timeout=120):
        tstart = self.get_sim_time()

        type_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

        self.change_mode('GUIDED')
        at_stopping_point = False

        # ModeGuided rejects targets outside the fence, so disable the fence
        # while sending the target and re-enable it once the autopilot has
        # accepted it.  Avoidance then runs against the saved target.
        self.do_fence_disable()
        self.mav.mav.set_position_target_global_int_send(
            0,
            target_system,
            target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask,
            int(loc.lat * 1.0e7),
            int(loc.lng * 1.0e7),
            0, # alt
            0, # x-ve
            0, # y-vel
            0, # z-vel
            0, # afx
            0, # afy
            0, # afz
            0, # yaw,
            0, # yaw-rate
        )
        self.do_fence_enable()

        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Did not arrive and stop at boundary")
            m = self.mav.recv_match(blocking=True,
                                    timeout=1)
            if m is None:
                continue
            t = m.get_type()
            if t == "POSITION_TARGET_GLOBAL_INT":
                print("Target: (%s)" % str(m))
            elif t == "GLOBAL_POSITION_INT":
                print("Position: (%s)" % str(m))
                delta = self.get_distance(
                    Location.latlon_only(m.lat * 1e-7, m.lon * 1e-7),
                    expected_stopping_point)
                print("delta: %s want_delta<%f" % (str(delta), expected_distance_epsilon))
                at_stopping_point = delta < expected_distance_epsilon
            elif t == "VFR_HUD":
                print("groundspeed: %f" % m.groundspeed)
                if at_stopping_point:
                    if m.groundspeed < 1:
                        self.progress("Seemed to have stopped at stopping point")
                        return

    def test_poly_fence_avoidance_dont_breach_exclusion(self, target_system=1, target_component=1):
        self.start_subtest("Ensure we stop before breaching an exclusion fence")
        here = self.get_location()
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 20), # br
                self.offset_location_ne(here, 50, 40), # tr
                self.offset_location_ne(here, -50, 40), # tl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # west
                self.offset_location_ne(here, -50, -20), # tl
                self.offset_location_ne(here, 50, -20), # tr
                self.offset_location_ne(here, 50, -40), # br
                self.offset_location_ne(here, -50, -40), # bl,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, {
                "radius": 30,
                "loc": self.offset_location_ne(here, -60, 0),
            }),
        ])
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "AVOID_ENABLE": 3,
            # this scenario sits outside the inclusion-circle fence, so the
            # vehicle is in breach the instant the fence is enabled.  We are
            # exercising avoidance (which keeps us off the exclusion boundary),
            # not the breach failsafe, so report-only keeps us in GUIDED rather
            # than being forced into HOLD when the fence is re-enabled.
            "FENCE_ACTION": 0,
        })
        fence_middle = self.offset_location_ne(here, 0, 30)
        # the fences above are placed relative to "here", so the point we
        # stop at is too: just short of the exclusion fence 20m to our
        # east.  This used to be a fixed location, which only worked
        # while "here" happened to be the startup location.
        expected_stopping_point = self.offset_location_ne(here, -1.05, 15.05)
        self.drive_somewhere_stop_at_boundary(
            fence_middle,
            expected_stopping_point,
            target_system=target_system,
            target_component=target_component,
            expected_distance_epsilon=3)
        self.set_parameter("AVOID_ENABLE", 0)
        self.do_RTL()

    @testcase(bucket="Rover")
    def PolyFenceAvoidance(self, target_system=1, target_component=1):
        '''PolyFence avoidance tests'''
        self.change_mode("LOITER")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode("MANUAL")
        self.reach_heading_manual(180, turn_right=False)
        self.change_mode("GUIDED")

        self.test_poly_fence_avoidance_dont_breach_exclusion(target_system=target_system, target_component=target_component)

        self.disarm_vehicle()
