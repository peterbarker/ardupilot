'''
QuadPlane autotest: MidAirDisarmDisallowed

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MidAirDisarmDisallowed(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def MidAirDisarmDisallowed(self):
        '''Check disarm behaviour in Q-mode'''
        self.start_subtest("Basic arm in qloiter")
        self.set_parameter("FLIGHT_OPTIONS", 0)
        self.change_mode('QLOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()

        self.context_push()
        self.start_subtest("Ensure disarming in q-modes on ground works")
        self.set_parameter("FLIGHT_OPTIONS", 1 << 11)
        self.arm_vehicle()
        self.disarm_vehicle()  # should be OK as we're not flying yet
        self.context_pop()

        self.start_subtest("Ensure no disarming mid-air")
        self.arm_vehicle()
        self.set_rc(3, 2000)
        self.wait_altitude(5, 50, relative=True)
        self.set_rc(3, 1000)
        disarmed = False
        try:
            self.disarm_vehicle()
            disarmed = True
        except ValueError as e:
            self.progress("Got %s" % repr(e))
            if "Expected MAV_RESULT_ACCEPTED got MAV_RESULT_FAILED" not in str(e):
                raise e
        if disarmed:
            raise NotAchievedException("Disarmed when we shouldn't have")

        self.change_mode('QLAND')
        self.wait_disarmed()

        self.start_subtest("Check we can disarm after a short period on the ground")
        self.takeoff(5, 'QHOVER')
        self.change_mode('QLAND')
        try:
            self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 10)
            self.wait_extended_sys_state(
                landed_state=mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                vtol_state=mavutil.mavlink.MAV_VTOL_STATE_MC,
                timeout=60
            )
        except Exception:
            self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 0)
            raise

        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, -1)
        self.disarm_vehicle()
