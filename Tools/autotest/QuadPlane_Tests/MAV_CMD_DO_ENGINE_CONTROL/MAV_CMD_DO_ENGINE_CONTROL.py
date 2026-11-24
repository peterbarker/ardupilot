'''
QuadPlane autotest: MAV_CMD_DO_ENGINE_CONTROL

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_ENGINE_CONTROL(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def MAV_CMD_DO_ENGINE_CONTROL(self):
        '''test MAV_CMD_DO_ENGINE_CONTROL mavlink command'''

        expected_idle_rpm_min = 65
        expected_idle_rpm_max = 75
        expected_starter_rpm_min = 345
        expected_starter_rpm_max = 355

        rc_engine_start_chan = 11
        self.setup_ICEngine_vehicle()

        self.wait_ready_to_arm()

        for method in self.run_cmd, self.run_cmd_int:
            self.change_mode('MANUAL')
            self.set_rc(rc_engine_start_chan, 1500)  # allow motor to run
            self.wait_rpm(1, 0, 0, minimum_duration=1)
            self.arm_vehicle()
            self.wait_rpm(1, 0, 0, minimum_duration=1)
            self.start_subtest("Start motor")
            method(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=1)
            self.wait_rpm(1, expected_starter_rpm_min, expected_starter_rpm_max)
            self.wait_rpm(1, expected_idle_rpm_min, expected_idle_rpm_max, minimum_duration=10)

            # starting the motor while it is running is failure
            # (probably wrong, but that's how this works):
            self.start_subtest("try start motor again")
            self.context_collect('STATUSTEXT')
            method(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=1, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
            self.wait_statustext("already running", check_context=True)
            self.context_stop_collecting('STATUSTEXT')
            # shouldn't affect run state:
            self.wait_rpm(1, expected_idle_rpm_min, expected_idle_rpm_max, minimum_duration=1)

            self.start_subtest("Stop motor")
            method(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=0)
            self.wait_rpm(1, 0, 0, minimum_duration=1)

            self.start_subtest("Stop motor (again)")
            method(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=0)
            self.wait_rpm(1, 0, 0, minimum_duration=1)

            self.start_subtest("Check start chan control disable")
            old_start_channel_value = self.get_rc_channel_value(rc_engine_start_chan)
            self.set_rc(rc_engine_start_chan, 1000)
            self.wait_rc_channel_value(rc_engine_start_chan, 1000, timeout=5)
            self.context_collect('STATUSTEXT')
            method(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=1, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
            self.wait_statustext("start control disabled", check_context=True)
            self.context_stop_collecting('STATUSTEXT')
            self.set_rc(rc_engine_start_chan, old_start_channel_value)
            self.wait_rpm(1, 0, 0, minimum_duration=1)

            self.start_subtest("test start-at-height")
            self.wait_rpm(1, 0, 0, minimum_duration=1)
            self.context_collect('STATUSTEXT')
            method(
                mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL,
                p1=1,  # start
                p3=15.5, # ... at 15.5 metres
            )
            self.wait_statustext("height set to 15.5m", check_context=True)
            self.wait_rpm(1, 0, 0, minimum_duration=2)

            self.takeoff(20, mode='GUIDED')
            self.wait_rpm(1, expected_starter_rpm_min, expected_starter_rpm_max, minimum_duration=1)
            self.wait_statustext("Engine running", check_context=True)
            self.context_stop_collecting('STATUSTEXT')

            # stop the motor again:
            method(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=0)
            self.wait_rpm(1, 0, 0, minimum_duration=1)

            self.change_mode('QLAND')
            self.wait_disarmed(timeout=45)
