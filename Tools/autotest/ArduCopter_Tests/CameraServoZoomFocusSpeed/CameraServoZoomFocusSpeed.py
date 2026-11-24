'''
Copter autotest: CameraServoZoomFocusSpeed

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CameraServoZoomFocusSpeed(TestGroup):
    @testcase(bucket="CopterTests2b")
    def CameraServoZoomFocusSpeed(self):
        '''test CAM1_ZOM_RAT_MAX and CAM1_FOC_RAT_MAX set the servo camera zoom and focus rates'''
        zoom_chan = 9
        focus_chan = 10
        zoom_function = 180  # CameraZoom
        focus_function = 92   # CameraFocus
        zoom_speed = 25  # %/s
        focus_speed = 40  # %/s
        start_pct = 0
        sample_s = 1
        rate_tolerance = 1.5  # %/s

        self.set_parameters({
            "CAM1_TYPE": 1,  # servo
            "CAM1_ZOM_RAT_MAX": zoom_speed,
            "CAM1_FOC_RAT_MAX": focus_speed,
            "SERVO%u_FUNCTION" % zoom_chan: zoom_function,
            "SERVO%u_FUNCTION" % focus_chan: focus_function,
        })
        self.reboot_sitl()  # needed for CAM1_TYPE to take effect

        pwm_min = self.get_parameter("SERVO%u_MIN" % zoom_chan)
        pwm_max = self.get_parameter("SERVO%u_MAX" % zoom_chan)
        pwm_span = pwm_max - pwm_min

        # start at 0% so a slightly long sample cannot hit a limit
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM,
            p1=mavutil.mavlink.ZOOM_TYPE_RANGE,
            p2=start_pct,
        )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS,
            p1=mavutil.mavlink.FOCUS_TYPE_RANGE,
            p2=start_pct,
        )

        # zoom in and focus out at full speed
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM,
            p1=mavutil.mavlink.ZOOM_TYPE_CONTINUOUS,
            p2=1,
        )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS,
            p1=mavutil.mavlink.FOCUS_TYPE_CONTINUOUS,
            p2=1,
        )

        # SERVO_OUTPUT_RAW.time_usec is AP_HAL::micros(), so the check does
        # not depend on how promptly the commands were handled
        self.drain_mav()
        first = self.assert_receive_message('SERVO_OUTPUT_RAW')
        self.delay_sim_time(sample_s, "camera zoom and focus to move")
        self.drain_mav()
        second = self.assert_receive_message('SERVO_OUTPUT_RAW')
        dt = (second.time_usec - first.time_usec) * 1.0e-6
        if dt < 0.8 * sample_s:
            raise NotAchievedException(
                "SERVO_OUTPUT_RAW samples too close together: want>=%fs got=%fs" % (0.8 * sample_s, dt))

        zoom_rate = (getattr(second, "servo%u_raw" % zoom_chan) -
                     getattr(first, "servo%u_raw" % zoom_chan)) * 100.0 / (pwm_span * dt)
        focus_rate = (getattr(second, "servo%u_raw" % focus_chan) -
                      getattr(first, "servo%u_raw" % focus_chan)) * 100.0 / (pwm_span * dt)
        if abs(zoom_rate - zoom_speed) > rate_tolerance:
            raise NotAchievedException(
                "zoom rate want=%f%%/s got=%f%%/s" % (zoom_speed, zoom_rate))
        if abs(focus_rate - focus_speed) > rate_tolerance:
            raise NotAchievedException(
                "focus rate want=%f%%/s got=%f%%/s" % (focus_speed, focus_rate))
