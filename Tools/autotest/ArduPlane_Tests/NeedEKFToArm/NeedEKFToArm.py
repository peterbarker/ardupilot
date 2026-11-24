'''
Plane autotest: NeedEKFToArm

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class NeedEKFToArm(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def NeedEKFToArm(self):
        """Ensure the EKF must be healthy for the vehicle to arm."""
        self.progress("Ensuring we need EKF to be healthy to arm")
        self.set_parameter("SIM_GPS1_ENABLE", 0)
        self.context_collect("STATUSTEXT")
        tstart = self.get_sim_time()
        success = False
        for run_cmd in self.run_cmd, self.run_cmd_int:
            while not success:
                if self.get_sim_time_cached() - tstart > 60:
                    raise NotAchievedException("Did not get correct failure reason")
                run_cmd(mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS)
                try:
                    self.wait_statustext(".*AHRS: not using configured AHRS type.*", timeout=1, check_context=True, regex=True)
                    success = True
                    continue
                except AutoTestTimeoutException:
                    pass

        self.set_parameter("SIM_GPS1_ENABLE", 1)
        self.wait_ready_to_arm()
