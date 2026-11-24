'''
Copter autotest: GuidedForceArm

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GuidedForceArm(TestGroup):
    @testcase(bucket="CopterTests2b")
    def GuidedForceArm(self):
        '''ensure Guided acts appropriately when force-armed'''
        self.set_parameters({
            "EK3_SRC2_VELXY": 5,
            "SIM_GPS1_ENABLE": 0,
        })
        self.load_default_params_file("copter-optflow.parm")
        self.reboot_sitl()
        self.delay_sim_time(30, reason="EKF to initialise with optical flow")
        self.change_mode('GUIDED')
        self.arm_vehicle(force=True)
        self.takeoff(20, mode='GUIDED')
        location = self.offset_location_ne(self.get_location('SIMSTATE'), metres_north=0, metres_east=-300)
        self.progress("Ensure we don't move for 10 seconds")
        tstart = self.get_sim_time()
        startpos = self.get_location('SIMSTATE')
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 10:
                break
            self.send_set_position_target_global_int(int(location.lat*1e7), int(location.lng*1e7), 10)
            dist = self.get_distance(startpos, self.get_location('SIMSTATE'))
            if dist > 10:
                raise NotAchievedException("Wandered too far from start position")
            self.delay_sim_time(1, reason="poll interval")

        self.disarm_vehicle(force=True)
        self.reboot_sitl()
