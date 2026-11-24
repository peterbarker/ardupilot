'''
Plane autotest: ForceArm

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class ForceArm(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def ForceArm(self):
        '''check force-arming functionality'''
        self.set_parameter("SIM_GPS1_ENABLE", 0)
        # 21196 is the mavlink standard, 2989 is legacy
        for magic_value in 21196, 2989:
            self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK,
                                   present=True,
                                   enabled=True,
                                   healthy=False,
                                   )
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                p1=1,  # ARM
                p2=0,
                want_result=mavutil.mavlink.MAV_RESULT_FAILED,
            )
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                p1=1,  # ARM
                p2=magic_value,
                want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
            )
            self.disarm_vehicle()
