'''
Rover autotest: MAV_GCS_ENFORCE

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import PreconditionFailedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_GCS_ENFORCE(TestGroup):
    @testcase(bucket="Rover")
    def MAV_GCS_ENFORCE(self):
        '''Test enforcement of strict mode for MAV_GCS_SYSID'''
        '''Run the same arming code with correct then incorrect SYSID'''

        if self.mav.source_system != self.mav.mav.srcSystem:
            raise PreconditionFailedException("Expected mav.source_system and mav.srcSystem to match")

        self.context_push()
        old_srcSystem = self.mav.mav.srcSystem
        ex = None
        try:
            self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)
            self.set_parameter("MAV_OPTIONS", 1, add_to_context=False)

            self.change_mode('MANUAL')

            self.progress("make sure I can arm ATM")
            self.wait_ready_to_arm()
            self.arm_vehicle(timeout=5)
            self.disarm_vehicle()

            self.do_timesync_roundtrip()

            # should not be able to arm from a system id which is not MY_SYSID
            self.progress("Attempting to arm vehicle from bad system-id")
            success = None
            try:
                # temporarily set a different system ID than normal:
                self.mav.mav.srcSystem = 72
                self.arm_vehicle(timeout=5)
                self.disarm_vehicle()
                success = False
            except AutoTestTimeoutException:
                success = True
            self.mav.mav.srcSystem = old_srcSystem
            if not success:
                raise NotAchievedException("Managed to arm with MAV_GCS enforce set")

            # should be able to arm from the vehicle's own components:
            self.progress("Attempting to arm vehicle from vehicle component")
            comp_arm_exception = None
            try:
                self.mav.mav.srcSystem = 1
                self.arm_vehicle(timeout=5)
                self.disarm_vehicle()
            except Exception as e:  # noqa: BLE001
                comp_arm_exception = e
            self.mav.mav.srcSystem = old_srcSystem
            if comp_arm_exception is not None:
                raise comp_arm_exception

        except Exception as e:  # noqa: BLE001
            self.print_exception_caught(e)
            ex = e
        self.mav.mav.srcSystem = old_srcSystem
        self.set_parameter("MAV_OPTIONS", 0, add_to_context=False)
        self.context_pop()
        if ex is not None:
            raise ex
