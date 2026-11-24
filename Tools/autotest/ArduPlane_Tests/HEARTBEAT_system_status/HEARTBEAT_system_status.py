'''
Plane autotest: HEARTBEAT_system_status

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class HEARTBEAT_system_status(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def HEARTBEAT_system_status(self):
        '''check HEARTBEAT system_status tracks the vehicle state'''
        def wait_system_status(status, timeout=30):
            self.wait_message_field_values('HEARTBEAT', {"system_status": status}, timeout=timeout)

        self.wait_ready_to_arm()
        wait_system_status(mavutil.mavlink.MAV_STATE_STANDBY)

        self.set_parameter("CRASH_DETECT", 1)  # disarm on crash
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 0, 50),
        ])
        self.wait_current_waypoint(2, timeout=60)
        wait_system_status(mavutil.mavlink.MAV_STATE_ACTIVE)

        self.start_subtest("failsafe")
        self.set_parameter("SIM_RC_FAIL", 1)
        wait_system_status(mavutil.mavlink.MAV_STATE_CRITICAL)
        self.set_parameter("SIM_RC_FAIL", 0)
        wait_system_status(mavutil.mavlink.MAV_STATE_ACTIVE)
        self.assert_mode_is('AUTO')

        self.start_subtest("crash")
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "SIM_ENGINE_FAIL": 1 << 2,  # throttle servo
            "SIM_ENGINE_MUL": 0,
        })
        self.wait_statustext("Crash detected", check_context=True, timeout=180)
        self.wait_disarmed()
        wait_system_status(mavutil.mavlink.MAV_STATE_EMERGENCY)

        self.reboot_sitl()
