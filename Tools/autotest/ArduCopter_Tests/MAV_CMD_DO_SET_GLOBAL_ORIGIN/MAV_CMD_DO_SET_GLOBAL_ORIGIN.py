'''
Copter autotest: MAV_CMD_DO_SET_GLOBAL_ORIGIN

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_SET_GLOBAL_ORIGIN(TestGroup):
    @testcase(bucket="CopterTests2d")
    def MAV_CMD_DO_SET_GLOBAL_ORIGIN(self):
        '''test MAV_CMD_DO_SET_GLOBAL_ORIGIN command'''
        # don't run this test if command not available:
        try:
            mavutil.mavlink.MAV_CMD_DO_SET_GLOBAL_ORIGIN
        except AttributeError:
            return

        # disable the GPS so we don't get origin that way:
        self.set_parameters({
            "SIM_GPS1_ENABLE": 0,
            "GPS1_TYPE": 0,
        })
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        # we must wait until we are using EKF3 as DCM does not allow
        # set_origin:
        self.wait_statustext("EKF3 active", check_context=True)
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_SET_GLOBAL_ORIGIN,
            p5=0,
            p6=0,
            p7=0,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED
        )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_SET_GLOBAL_ORIGIN,
            p5=0,
            p6=int(214*1e7),
            p7=0,
            want_result=mavutil.mavlink.MAV_RESULT_DENIED
        )
        origin_lat = -23.322332
        origin_lng = 123.45678
        origin_alt = 23.67  # metres
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_SET_GLOBAL_ORIGIN,
            p5=int(origin_lat*1e7),
            p6=int(origin_lng*1e7),
            p7=origin_alt
        )
        ggo = self.poll_message('GPS_GLOBAL_ORIGIN')
        self.assert_message_field_values(ggo, {
            "latitude": int(origin_lat*1e7),
            "longitude": int(origin_lng*1e7),
            "altitude": int(origin_alt*1000),  # m -> mm
        })
