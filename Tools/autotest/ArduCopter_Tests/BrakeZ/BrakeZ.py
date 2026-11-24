'''
Copter autotest: BrakeZ

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class BrakeZ(TestGroup):
    @testcase(bucket="CopterTests2b")
    def BrakeZ(self):
        '''check jerk limit correct in Brake mode'''
        self.set_parameter('PSC_D_JERK', 3)
        self.takeoff(50, mode='GUIDED')
        vx, vy, vz_up = (0, 0, -1)
        self.test_guided_local_velocity_target(vx=vx, vy=vy, vz_up=vz_up, timeout=10)

        self.wait_for_local_velocity(vx=vx, vy=vy, vz_up=vz_up, timeout=10)
        self.change_mode('BRAKE')
        self.wait_for_local_velocity(vx=0, vy=0, vz_up=0, timeout=10)
        self.land_and_disarm()
