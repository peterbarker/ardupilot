'''
Sub autotest: SetGlobalOrigin

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SetGlobalOrigin(TestGroup):
    def collected_msgs(self, msg_type):
        c = self.context_get()
        if msg_type not in c.collections:
            raise NotAchievedException(f"Not collecting ({msg_type})")
        return c.collections[msg_type]

    @testcase(bucket="Sub")
    def SetGlobalOrigin(self):
        """Test SET_GPS_GLOBAL_ORIGIN mav msg"""
        self.set_parameters({
            'GPS1_TYPE': 0,             # Disable the GPS
            'EK3_SRC1_POSXY': 0,        # Make sure EK3_SRC parameters do not refer to a GPS
        })
        self.reboot_sitl()

        # Wait for the EKF to be happy in constant position mode
        self.wait_ready_to_arm_const_pos()

        if self.current_onboard_log_contains_message('ORGN'):
            raise NotAchievedException("Found unexpected ORGN message")

        self.context_collect('GPS_GLOBAL_ORIGIN')

        # This should set the EKF origin, write an ORGN msg to df and a GPS_GLOBAL_ORIGIN msg to MAVLink
        self.mav.mav.set_gps_global_origin_send(1, int(47.607584 * 1e7), int(-122.343911 * 1e7), 0)
        self.delay_sim_time(2, reason="EKF origin to be set")

        if not self.current_onboard_log_contains_message('ORGN'):
            raise NotAchievedException("Did not find expected ORGN message")

        num_mavlink_origin_msgs = len(self.collected_msgs('GPS_GLOBAL_ORIGIN'))
        if num_mavlink_origin_msgs != 1:
            raise NotAchievedException(f"Expected 1 GPS_GLOBAL_ORIGIN message, found {num_mavlink_origin_msgs}")
