'''
Copter autotest: ReplayOriginFrame

AP_FLAKE8_CLEAN
'''

import time

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ReplayOriginFrame(TestGroup):
    @testcase(bucket="CopterTests2b")
    def ReplayOriginFrame(self):
        '''check an EKF origin set from a GCS is logged after the current replay frame'''
        # Replay runs each frame's EKF update when it reaches the
        # frame's RFRF.  An origin set after the update must be
        # logged after that RFRF, or Replay applies it before the
        # update, which the origin can change (e.g. the earth
        # magnetic field used by magnetometer fusion).
        self.wait_ready_to_arm()
        origin = self.assert_receive_message('GLOBAL_POSITION_INT')

        # with no GPS the vehicle has no origin until we set one
        self.set_parameters({
            "GPS1_TYPE": 0,
            "EK2_ENABLE": 1,
            "LOG_REPLAY": 1,
            "LOG_DISARMED": 1,
        })
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_LOGGING, True, True, True)
        log_filepath = self.current_onboard_log_filepath()

        # without a GPS or some sort of external prompting, AP
        # doesn't send system_time messages.  So prompt it:
        self.mav.mav.system_time_send(int(time.time() * 1000000), 0)
        self.delay_sim_time(5, reason="EKFs to initialise")
        self.mav.mav.set_gps_global_origin_send(1, origin.lat, origin.lon, origin.alt)
        self.wait_statustext("EKF3 IMU0 origin set", check_context=True)
        self.wait_statustext("EKF2 IMU0 origin set", check_context=True)
        self.delay_sim_time(5, reason="log to be written")
        self.reboot_sitl()

        dfreader = self.dfreader_for_path(log_filepath)
        frame_open = False
        counts = {}
        in_frame = {}
        while True:
            m = dfreader.recv_match(type=['RFRH', 'RFRF', 'RSO2', 'RSO3'])
            if m is None:
                break
            mtype = m.get_type()
            if mtype == 'RFRH':
                frame_open = True
            elif mtype == 'RFRF':
                frame_open = False
            else:
                counts[mtype] = counts.get(mtype, 0) + 1
                if frame_open:
                    in_frame[mtype] = in_frame.get(mtype, 0) + 1
        self.progress("Origin messages: %s, logged inside a frame: %s" % (str(counts), str(in_frame)))
        for mtype in 'RSO2', 'RSO3':
            if counts.get(mtype, 0) == 0:
                raise NotAchievedException("No %s in log" % mtype)
        if len(in_frame):
            raise NotAchievedException("Origin logged before the end of the current frame: %s" % str(in_frame))
