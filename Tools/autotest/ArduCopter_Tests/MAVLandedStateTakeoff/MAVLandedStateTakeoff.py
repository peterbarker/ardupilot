'''
Copter autotest: MAVLandedStateTakeoff

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAVLandedStateTakeoff(TestGroup):
    @testcase(bucket="CopterTests1a")
    def MAVLandedStateTakeoff(self):
        '''check EXTENDED_SYS_STATE message'''
        ex = None
        try:
            self.set_message_rate_hz(id=mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, rate_hz=1)
            self.wait_extended_sys_state(vtol_state=mavutil.mavlink.MAV_VTOL_STATE_MC,
                                         landed_state=mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, timeout=10)
            self.load_mission(filename="copter_mission.txt")
            self.set_parameter(name="AUTO_OPTIONS", value=3)
            self.change_mode(mode="AUTO")
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.wait_extended_sys_state(vtol_state=mavutil.mavlink.MAV_VTOL_STATE_MC,
                                         landed_state=mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF, timeout=30)
            self.wait_extended_sys_state(vtol_state=mavutil.mavlink.MAV_VTOL_STATE_MC,
                                         landed_state=mavutil.mavlink.MAV_LANDED_STATE_IN_AIR, timeout=60)
            self.land_and_disarm()
        except Exception as e:  # noqa: BLE001
            self.print_exception_caught(e)
            ex = e
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, -1)
        if ex is not None:
            raise ex
