'''
Rover autotest: PolyFenceObjectAvoidanceBendyRulerEasierGuided

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location

from test_discovery import TestGroup
from test_discovery import testcase


class PolyFenceObjectAvoidanceBendyRulerEasierGuided(TestGroup):
    @testcase(bucket="Rover")
    def PolyFenceObjectAvoidanceBendyRulerEasierGuided(self, target_system=1, target_component=1):
        '''finish-line issue means we can't complete the harder one.  This
        test can go away once we've nailed that one.  The only
        difference here is the target point.
        '''
        self.load_fence_using_mavwp("rover-path-bendyruler-fence.txt")
        self.set_parameters({
            "AVOID_ENABLE": 3,
            "OA_TYPE": 1,
            "FENCE_ENABLE": 1,
            "WP_RADIUS": 5,
        })
        self.reboot_sitl()
        self.set_parameters({
            "OA_BR_LOOKAHEAD": 60,
        })
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        target_loc = Location(40.071260, -105.227000, 1584, AltFrame.ABSOLUTE)
        self.send_guided_mission_item(target_loc,
                                      target_system=target_system,
                                      target_component=target_component)
        # FIXME: we don't get within WP_RADIUS of our target?!
        self.wait_location(target_loc, timeout=300, accuracy=15)
        self.do_RTL(timeout=300)
        self.disarm_vehicle()
