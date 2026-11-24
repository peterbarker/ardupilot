'''
Rover autotest: PolyFenceObjectAvoidanceGuided

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import Location

from test_discovery import TestGroup
from test_discovery import testcase


class PolyFenceObjectAvoidanceGuided(TestGroup):
    @testcase(bucket="Rover")
    def PolyFenceObjectAvoidanceGuided(self, target_system=1, target_component=1):
        '''PolyFence object avoidance tests - guided mode'''
        if not self.mavproxy_can_do_mision_item_protocols():
            return

        self.test_poly_fence_object_avoidance_guided_pathfinding(
            target_system=target_system,
            target_component=target_component)
        self.test_poly_fence_object_avoidance_guided_two_squares(
            target_system=target_system,
            target_component=target_component)

    def test_poly_fence_object_avoidance_guided_pathfinding(self, target_system=1, target_component=1):
        self.load_fence("rover-path-planning-fence.txt")
        self.set_parameters({
            "AVOID_ENABLE": 3,
            "OA_TYPE": 2,
            "FENCE_MARGIN": 0, # FIXME: https://github.com/ArduPilot/ardupilot/issues/11601
        })
        self.reboot_sitl()
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter("FENCE_ENABLE", 1)
        target_loc = Location.latlon_only(40.073800, -105.229172)
        self.send_guided_mission_item(target_loc,
                                      target_system=target_system,
                                      target_component=target_component)
        self.wait_location(target_loc, height_accuracy=None, timeout=300)
        self.do_RTL(timeout=300)
        self.disarm_vehicle()

    def test_poly_fence_object_avoidance_guided_two_squares(self, target_system=1, target_component=1):
        self.start_subtest("Ensure we can steer around obstacles in guided mode")
        here = self.get_location()
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # east
                self.offset_location_ne(here, -50, 20), # bl
                self.offset_location_ne(here, 50, 10), # tl
                self.offset_location_ne(here, 50, 30), # tr
                self.offset_location_ne(here, -50, 40), # br,
            ]),
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                # further east (and south
                self.offset_location_ne(here, -60, 60), # bl
                self.offset_location_ne(here, 40, 70), # tl
                self.offset_location_ne(here, 40, 90), # tr
                self.offset_location_ne(here, -60, 80), # br,
            ]),
        ])
        self.context_push()
        ex = None
        try:
            self.set_parameters({
                "AVOID_ENABLE": 3,
                "OA_TYPE": 2,
            })
            self.reboot_sitl()
            self.change_mode('GUIDED')
            self.wait_ready_to_arm()
            self.set_parameter("FENCE_ENABLE", 1)
            self.arm_vehicle()

            self.change_mode("GUIDED")
            target = Location.latlon_only(40.071382, -105.228340)
            self.send_guided_mission_item(target,
                                          target_system=target_system,
                                          target_component=target_component)
            self.wait_location(target, height_accuracy=None, timeout=300)
            self.do_RTL()
            self.disarm_vehicle()
        except Exception as e:  # noqa: BLE001
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex
