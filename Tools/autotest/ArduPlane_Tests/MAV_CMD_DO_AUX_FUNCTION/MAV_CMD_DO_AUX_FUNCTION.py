'''
Plane autotest: MAV_CMD_DO_AUX_FUNCTION

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_AUX_FUNCTION(TestGroup):
    def _MAV_CMD_DO_AUX_FUNCTION(self, run_cmd):
        '''Test triggering Auxiliary Functions via mavlink'''
        self.context_collect('STATUSTEXT')
        self.run_auxfunc(64, 2, run_cmd=run_cmd)  # 64 == reverse throttle
        self.wait_statustext("RevThrottle: ENABLE", check_context=True)
        self.run_auxfunc(64, 0, run_cmd=run_cmd)
        self.wait_statustext("RevThrottle: DISABLE", check_context=True)
        self.run_auxfunc(65, 2, run_cmd=run_cmd)  # 65 == GPS_DISABLE
        # ... and put it back.  Aux function state is not a parameter, so
        # context_pop() does not restore it, and the suite no longer
        # restarts the SITL between tests - so without this every test
        # which follows on this worker inherits a vehicle with no GPS,
        # and an EKF which never starts:
        #     AHRS2Logging (...) (Failed to get EKF.flags=831)      [EKF_UNINITIALIZED]
        #     AHRS_ORIENTATION (...) (GPS status bits did not become good)
        self.run_auxfunc(65, 0, run_cmd=run_cmd)
        self.wait_gps_sys_status_not_present_or_enabled_and_healthy()

        self.start_subtest("Bad auxfunc")
        self.run_auxfunc(
            65231,
            2,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
            run_cmd=run_cmd,
        )

        self.start_subtest("Bad switchpos")
        self.run_auxfunc(
            62,
            17,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
            run_cmd=run_cmd,
        )

    @testcase(bucket="PlaneTests1a")
    def MAV_CMD_DO_AUX_FUNCTION(self):
        '''Test triggering Auxiliary Functions via mavlink'''
        self._MAV_CMD_DO_AUX_FUNCTION(run_cmd=self.run_cmd)
        self._MAV_CMD_DO_AUX_FUNCTION(run_cmd=self.run_cmd_int)
