'''
QuadPlane autotest: EXTENDED_SYS_STATE

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class EXTENDED_SYS_STATE(TestGroup):
    def EXTENDED_SYS_STATE_SLT(self):
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 10)
        self.change_mode("QHOVER")
        self.assert_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_MC,
                                       mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND)
        self.change_mode("FBWA")
        self.assert_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_FW,
                                       mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND)
        self.change_mode("QHOVER")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # should not change just because we arm:
        self.assert_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_MC,
                                       mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND)
        self.change_mode("MANUAL")
        self.assert_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_FW,
                                       mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND)
        self.change_mode("QHOVER")

        self.progress("Taking off")
        self.set_rc(3, 1750)
        self.wait_altitude(1, 5, relative=True)
        self.assert_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_MC,
                                       mavutil.mavlink.MAV_LANDED_STATE_IN_AIR)
        self.wait_altitude(10, 15, relative=True)

        self.progress("Transitioning to fixed wing")
        self.change_mode("FBWA")
        self.set_rc(3, 1900) # apply spurs
        self.wait_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_FW,
                                     mavutil.mavlink.MAV_LANDED_STATE_IN_AIR)
        self.wait_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_FW,
                                     mavutil.mavlink.MAV_LANDED_STATE_IN_AIR)

        # fly away from home so QRTL has a full landing approach
        self.wait_distance_to_home(400, 1000, timeout=60)

        # verify vtol_state goes FW -> TRANSITION_TO_MC -> MC during QRTL
        # with nothing else in between.
        self.progress("Commanding QRTL and collecting vtol_state transitions")
        self.context_push()
        self.context_collect('EXTENDED_SYS_STATE')
        self.set_rc(3, 1500)
        self.change_mode("QRTL")
        self.wait_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_MC,
                                     mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
                                     timeout=60)
        # Save the sequence of states during the approach to check sequencing later
        messages = self.context_collection('EXTENDED_SYS_STATE')
        self.context_pop()
        self.wait_extended_sys_state(mavutil.mavlink.MAV_VTOL_STATE_MC,
                                     mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                     timeout=60)
        self.wait_disarmed(timeout=60)

        # Check that the sequence of vtol states during that landing was correct.
        vtol_states = []
        for m in messages:
            if not vtol_states or vtol_states[-1] != m.vtol_state:
                vtol_states.append(m.vtol_state)

        expected = [
            mavutil.mavlink.MAV_VTOL_STATE_FW,
            mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_MC,
            mavutil.mavlink.MAV_VTOL_STATE_MC,
        ]

        state_names = [self.vtol_state_name(s) for s in vtol_states]
        self.progress("vtol_state sequence: %s" % state_names)

        if vtol_states != expected:
            expected_names = [self.vtol_state_name(s) for s in expected]
            raise NotAchievedException(
                "Unexpected vtol_state sequence during QRTL landing: "
                "got %s expected %s" % (state_names, expected_names))

    @testcase(bucket="QuadPlaneTests1c")
    def EXTENDED_SYS_STATE(self):
        '''Check extended sys state works'''
        self.EXTENDED_SYS_STATE_SLT()
