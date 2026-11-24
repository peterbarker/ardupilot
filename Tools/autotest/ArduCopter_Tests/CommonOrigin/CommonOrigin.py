'''
Copter autotest: CommonOrigin

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CommonOrigin(TestGroup):
    @testcase(bucket="CopterTests2d")
    def CommonOrigin(self):
        """Test common origin between EKF2 and EKF3"""
        # start on EKF2
        self.set_parameters({
            'EK2_ENABLE': 1,
            'EK3_CHECK_SCALE': 1, # make EK3 slow to get origin
        })
        # collect before the reboot: the messages waited for below are
        # emitted as the vehicle comes up, so a collection started
        # afterwards is created too late to catch them.  reboot_sitl()
        # empties collections as it goes, so the identical messages
        # from before the reboot cannot satisfy those waits.
        self.context_collect('STATUSTEXT')

        self.reboot_sitl()

        self.wait_statustext("EKF2 IMU0 origin set", timeout=60, check_context=True)
        self.wait_statustext("EKF2 IMU0 is using GPS", timeout=60, check_context=True)

        # "AHRS: ... active" is emitted only when the active backend
        # changes, so ask for EKF2 here rather than before the reboot:
        # a vehicle which boots already configured for it comes up with
        # it active and says nothing, and the wait would be left
        # matching the message from before the reboot.
        self.set_parameter('AHRS_EKF_TYPE', 2)
        self.wait_statustext("EKF2 active", timeout=60, check_context=True)

        # get EKF2 origin
        ek2_origin = self.poll_message('GPS_GLOBAL_ORIGIN')

        # switch to EKF3
        self.set_parameters({
            'SIM_GPS1_GLTCH_X' : 0.001, # about 100m
            'EK3_CHECK_SCALE' : 100,
            'AHRS_EKF_TYPE'   : 3})

        self.wait_statustext("EKF3 IMU0 is using GPS", timeout=60, check_context=True)
        self.wait_statustext("EKF3 active", timeout=60, check_context=True)

        ek3_origin = self.poll_message('GPS_GLOBAL_ORIGIN')

        self.progress("Checking origins")
        if ek2_origin.time_usec == ek3_origin.time_usec:
            raise NotAchievedException("Did not get a new GPS_GLOBAL_ORIGIN message")

        print(ek2_origin, ek3_origin)

        if (ek2_origin.latitude != ek3_origin.latitude or
                ek2_origin.longitude != ek3_origin.longitude or
                ek2_origin.altitude != ek3_origin.altitude):
            raise NotAchievedException("Did not get matching EK2 and EK3 origins")
