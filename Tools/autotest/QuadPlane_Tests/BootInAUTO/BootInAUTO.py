'''
QuadPlane autotest: BootInAUTO

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class BootInAUTO(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def BootInAUTO(self):
        '''Test behaviour when booting in auto'''
        self.load_mission("mission.txt")
        self.set_parameters({
        })
        self.set_rc(5, 1000)
        self.wait_mode('AUTO')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.wait_current_waypoint(1, timeout=30)
        self.arm_vehicle()
        self.wait_altitude(9, 11, relative=True)  # value from mission file is 10
        distance = self.distance_to_home()
        # this distance check is very, very loose.  At time of writing
        # the vehicle actually pitches ~6 degrees on trakeoff,
        # wandering over 1m.
        if distance > 2:
            raise NotAchievedException("wandered from home (distance=%f)" %
                                       (distance,))
        self.change_mode('QLAND')
        self.wait_disarmed(timeout=60)
