'''
Rover autotest: PolyFenceObjectAvoidanceAuto

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import Location

from test_discovery import TestGroup
from test_discovery import testcase


class PolyFenceObjectAvoidanceAuto(TestGroup):
    @testcase(bucket="Rover")
    def PolyFenceObjectAvoidanceAuto(self, target_system=1, target_component=1):
        '''PolyFence object avoidance tests - auto mode'''
        mavproxy = self.start_mavproxy()
        self.load_fence_using_mavproxy(mavproxy, "rover-path-planning-fence.txt")
        self.stop_mavproxy(mavproxy)
        # self.load_fence("rover-path-planning-fence.txt")
        self.load_mission("rover-path-planning-mission.txt")
        self.set_parameters({
            "AVOID_ENABLE": 3,
            "OA_TYPE": 2,
            "FENCE_MARGIN": 0, # FIXME: https://github.com/ArduPilot/ardupilot/issues/11601
        })
        self.reboot_sitl()
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter("FENCE_ENABLE", 1)
        # target_loc is copied from the mission file
        target_loc = Location.latlon_only(40.073799, -105.229156)
        self.wait_location(target_loc, height_accuracy=None, timeout=300)
        # mission has RTL as last item
        self.wait_distance_to_home(3, 7, timeout=300)
        self.disarm_vehicle()
