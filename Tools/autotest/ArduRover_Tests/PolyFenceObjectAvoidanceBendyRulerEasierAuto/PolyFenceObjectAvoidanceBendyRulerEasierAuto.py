'''
Rover autotest: PolyFenceObjectAvoidanceBendyRulerEasierAuto

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location

from test_discovery import TestGroup
from test_discovery import testcase


class PolyFenceObjectAvoidanceBendyRulerEasierAuto(TestGroup):
    @testcase(bucket="Rover")
    def PolyFenceObjectAvoidanceBendyRulerEasierAuto(self, target_system=1, target_component=1):
        '''finish-line issue means we can't complete the harder one.  This
        test can go away once we've nailed that one.  The only
        difference here is the target point.
        '''
        self.load_fence_using_mavwp("rover-path-bendyruler-fence.txt")
        self.load_mission("rover-path-bendyruler-mission-easier.txt")

        self.set_parameters({
            "AVOID_ENABLE": 3,
            "OA_TYPE": 1,  # BendyRuler
            "FENCE_ENABLE": 1,
            "WP_RADIUS": 5,
        })
        self.reboot_sitl()
        self.set_parameters({
            "OA_BR_LOOKAHEAD": 60,
        })
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        target_loc = Location(40.071260, -105.227000, 1584, AltFrame.ABSOLUTE)
        # target_loc is copied from the mission file
        self.wait_location(target_loc, timeout=300)
        # mission has RTL as last item
        self.wait_distance_to_home(3, 7, timeout=300)
        self.disarm_vehicle()
