'''
Plane autotest: MAV_CMD_PREFLIGHT_CALIBRATION

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_PREFLIGHT_CALIBRATION(TestGroup):
    def _MAV_CMD_PREFLIGHT_CALIBRATION(self, command):
        self.context_push()
        self.start_subtest("Denied when armed")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p1=1,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )
        self.disarm_vehicle()

        self.context_collect('STATUSTEXT')

        self.start_subtest("gyro cal")
        command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p1=1,
        )

        self.start_subtest("baro cal")
        command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p3=1,
        )
        self.wait_statustext('Barometer calibration complete', check_context=True)

        # accelcal skipped here, it is checked elsewhere

        self.start_subtest("ins trim")
        command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=2,
        )

        # enforced delay between cals:
        self.delay_sim_time(5, reason="mandatory delay between calibrations")

        self.start_subtest("simple accel cal")
        command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=4,
        )
        # simple gyro cal makes the GPS units go unhealthy as they are
        # not maintaining their update rate (gyro cal is synchronous
        # in the main loop).  Usually ~30 seconds to recover...
        self.wait_gps_sys_status_not_present_or_enabled_and_healthy(timeout=60)

        self.start_subtest("force save accels")
        command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=76,
        )

        self.start_subtest("force save compasses")
        command(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p2=76,
        )

        self.context_pop()

    @testcase(bucket="PlaneTests1a")
    def MAV_CMD_PREFLIGHT_CALIBRATION(self):
        '''test MAV_CMD_PREFLIGHT_CALIBRATION mavlink handling'''
        self._MAV_CMD_PREFLIGHT_CALIBRATION(self.run_cmd)
        self._MAV_CMD_PREFLIGHT_CALIBRATION(self.run_cmd_int)
