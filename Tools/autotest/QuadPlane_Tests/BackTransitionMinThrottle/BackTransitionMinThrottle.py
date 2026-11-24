'''
QuadPlane autotest: BackTransitionMinThrottle

AP_FLAKE8_CLEAN
'''

import operator

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class BackTransitionMinThrottle(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def BackTransitionMinThrottle(self):
        '''Ensure min throttle is applied during back transition.'''
        wps = self.create_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.check_mission_upload_download(wps)
        self.set_parameter('Q_RTL_MODE', 1)

        trim_pwm = 1000 + 10*self.get_parameter("TRIM_THROTTLE")
        min_pwm = 1000 + 10*self.get_parameter("THR_MIN")

        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()
        self.context_collect('STATUSTEXT')

        self.wait_statustext("VTOL airbrake", check_context=True, timeout=300)
        self.wait_servo_channel_value(3, trim_pwm, comparator=operator.le, timeout=1)

        self.wait_statustext("VTOL position1", check_context=True, timeout=10)
        self.wait_servo_channel_value(3, min_pwm+10, comparator=operator.le, timeout=1)

        self.wait_disarmed(timeout=60)
