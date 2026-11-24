'''
Copter autotest: DroneCANCompass

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class DroneCANCompass(TestGroup):
    def yaw_error_deg(self):
        """return how far the estimated yaw is from the simulation's truth"""
        msgs = self.get_messages_frame(['ATTITUDE', 'SIMSTATE'])
        want = math.degrees(msgs['SIMSTATE'].yaw)
        got = math.degrees(msgs['ATTITUDE'].yaw)
        error = abs(mavextra.angle_diff(want, got))
        self.progress("yaw want=%f got=%f error=%f" % (want, got, error))
        return error

    # anomalous: ~12 minutes on CI against ~30s on a desktop; to be investigated
    @testcase(bucket="CopterTests1b")
    def DroneCANCompass(self):
        '''check the compass in a simulated DroneCAN peripheral'''
        # the peripheral is a separate device to the autopilot it is
        # speaking to, so its compass is not mounted in the same
        # orientation.  periph-compass.parm gives the peripheral's own
        # SIM_MAG1_ORIENT; COMPASS_ORIENT is what we tell the autopilot
        # about it.  The two should cancel, leaving the yaw correct.
        peripheral_orientation = 2  # 2 is ROTATION_YAW_90

        self.context_push()
        self.set_parameters({
            "CAN_P1_DRIVER": 1,
            # no directly-attached compasses, so the peripheral's is left
            # to supply us with a field:
            "SIM_MAG1_DEVID": 0,
            "SIM_MAG2_DEVID": 0,
            "SIM_MAG3_DEVID": 0,
            "COMPASS_USE2": 0,
            "COMPASS_USE3": 0,
            "COMPASS_ORIENT": peripheral_orientation,
        })
        # customisations=[] so that SITL is restarted: CAN_P1_DRIVER only
        # takes effect on a reboot, and the periph must not be spawned
        # until the vehicle is up and publishing the multicast sim state.
        self.restart_SITL_frame('copter-periph-compass', customisations=[])

        self.wait_ready_to_arm()
        error = self.yaw_error_deg()
        if error > 10:
            raise NotAchievedException("Yaw bad with orientations agreeing (error=%f)" % error)

        self.start_subtest("mis-describe the peripheral's mounting")
        # if the peripheral's orientation did not reach us through its
        # DroneCAN compass then lying about it here would change nothing:
        self.set_parameter("COMPASS_ORIENT", 0)  # 0 is ROTATION_NONE
        self.reboot_sitl()
        self.wait_ready_to_arm()

        error = self.yaw_error_deg()
        if error < 45:
            raise NotAchievedException("Yaw unaffected by the peripheral's orientation (error=%f)" % error)

        self.context_pop()
