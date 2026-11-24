'''
Copter autotest: GPS_INPUT

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

import vehicle_test_suite

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPS_INPUT(TestGroup):
    @testcase(bucket="CopterTests1c")
    def GPS_INPUT(self):
        '''Test GPS data injected via the GPS_INPUT MAVLink message (GPS_TYPE=MAV)'''
        # feed the first GPS instance over MAVLink rather than a simulated
        # serial backend.  Disable the simulated serial GPS so the only GPS
        # data ArduPilot sees is what we inject below:
        self.set_parameters({
            "GPS1_TYPE": 14,        # MAV
            "SIM_GPS1_ENABLE": 0,   # no simulated serial GPS
        })
        self.reboot_sitl()

        # we will echo the simulator's true state back as GPS_INPUT, so make
        # sure SIM_STATE (which carries truth lat/lng/alt/velocity) is streamed:
        self.context_set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE, 10)

        class GPSInputFeeder(vehicle_test_suite.TestSuite.MessageHook):
            '''emit a GPS_INPUT message for each SIM_STATE, pretending to be a
            companion computer supplying the GPS solution'''
            def __init__(self, suite):
                super().__init__(suite)
                self.count = 0

            def progress_prefix(self):
                return "GPSIN: "

            def process(self, mav, m):
                if m.get_type() != 'SIM_STATE':
                    return
                self.count += 1
                now_s = self.suite.get_sim_time_cached()
                # an arbitrary but valid GPS week and ms-of-week; ArduPilot
                # only requires week>0 and fix>=3 to apply jitter correction:
                time_week = 2345
                time_week_ms = int(now_s * 1000) % (7 * 86400 * 1000)
                mav.mav.gps_input_send(
                    int(now_s * 1e6),   # time_usec
                    0,                  # gps_id (first instance)
                    0,                  # ignore_flags: 0 == every field is valid
                    time_week_ms,
                    time_week,
                    3,                  # fix_type: 3D fix
                    m.lat_int,          # 1e7 degrees
                    m.lon_int,          # 1e7 degrees
                    m.alt,              # metres
                    1.0,                # hdop
                    1.5,                # vdop
                    m.vn, m.ve, m.vd,   # NED velocity, m/s
                    0.2,                # speed_accuracy, m/s
                    0.5,                # horiz_accuracy, m
                    0.8,                # vert_accuracy, m
                    15,                 # satellites_visible
                    0,                  # yaw (0 == not provided)
                )

        feeder = GPSInputFeeder(self)
        self.install_message_hook_context(feeder)

        # confirm ArduPilot consumed the injected GPS:
        self.wait_gps_fix_type_gte(3, timeout=60, verbose=True)
        self.wait_gps_satellite_count("GPS_RAW_INT", 15, timeout=30)

        # the EKF should accept the MAVLink GPS and allow arming:
        self.wait_ready_to_arm()

        # the estimated position should track the truth we are feeding in:
        self.install_message_hook_context(
            vehicle_test_suite.TestSuite.ValidateGlobalPositionIntAgainstSimState(self, max_allowed_divergence=25))

        # and we should be able to fly under position control:
        self.takeoff(10, mode='GUIDED')
        self.fly_guided_move_local(20, 0, 10)
        self.do_RTL()

        if feeder.count == 0:
            raise NotAchievedException("Never sent any GPS_INPUT messages")
        self.progress("Sent %u GPS_INPUT messages" % feeder.count)
