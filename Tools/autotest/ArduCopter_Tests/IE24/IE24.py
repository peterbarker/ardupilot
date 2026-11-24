'''
Copter autotest: IE24

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class IE24(TestGroup):
    @testcase(bucket="CopterTests1c")
    def IE24(self):
        '''Test IntelligentEnergy 2.4kWh generator with V1 and V2 telemetry protocols'''
        protocol_ver = (1, 2)
        for ver in protocol_ver:
            self.start_subtest(f"IE24 general tests for protocol {ver}")
            self.context_push()
            self.run_IE24(ver)
            self.context_pop()
            self.reboot_sitl()

            self.start_subtest(f"IE24 low capacity failsafe test for protocol {ver}")
            self.context_push()
            self.test_IE24_low_capacity_failsafe(ver)
            self.context_pop()
            self.reboot_sitl()

    def run_IE24(self, proto_ver):
        '''Test IntelligentEnergy 2.4kWh generator'''
        elec_battery_instance = 2
        fuel_battery_instance = 1
        self.set_parameters({
            "SERIAL5_PROTOCOL": 30,
            "SERIAL5_BAUD": 115200,
            "GEN_TYPE": 2,
            "BATT%u_MONITOR" % (fuel_battery_instance + 1): 18,  # fuel-based generator
            "BATT%u_MONITOR" % (elec_battery_instance + 1): 17,
            "SIM_IE24_ENABLE": proto_ver,
            "LOG_DISARMED": 1,
        })

        self.customise_SITL_commandline(["--serial5=sim:ie24"])

        self.start_subtest("Protocol %i: ensure that BATTERY_STATUS for electrical generator message looks right" % proto_ver)
        self.start_subsubtest("Protocol %i: Checking original voltage (electrical)" % proto_ver)
        # ArduPilot spits out essentially uninitialised battery
        # messages until we read things from the battery:
        original_elec_m = self.wait_message_field_values('BATTERY_STATUS', {
            "charge_state": mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_OK
        }, instance=elec_battery_instance, timeout=60)

        if original_elec_m.battery_remaining < 90:
            raise NotAchievedException("Bad original percentage")
        self.start_subsubtest("Ensure percentage is counting down")
        self.wait_message_field_values('BATTERY_STATUS', {
            "battery_remaining": original_elec_m.battery_remaining - 1,
        }, instance=elec_battery_instance)

        self.start_subtest("Protocol %i: ensure that BATTERY_STATUS for fuel generator message looks right" % proto_ver)
        self.start_subsubtest("Protocol %i: Checking original voltage (fuel)" % proto_ver)
        # ArduPilot spits out essentially uninitialised battery
        # messages until we read things from the battery:
        original_fuel_m = self.wait_message_field_values('BATTERY_STATUS', {
            "charge_state": mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_OK
        }, instance=fuel_battery_instance)
        if original_fuel_m.battery_remaining <= 90:
            raise NotAchievedException("Bad original percentage (want=>%f got %f" % (90, original_fuel_m.battery_remaining))
        self.start_subsubtest("Protocol %i: Ensure percentage is counting down" % proto_ver)
        self.wait_message_field_values('BATTERY_STATUS', {
            "battery_remaining": original_fuel_m.battery_remaining - 2,
        }, instance=fuel_battery_instance, timeout=30)

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()

        # Test for pre-arm check fail when state is not running
        self.start_subtest("Protocol %i: Without takeoff generator error should cause failsafe and disarm" % proto_ver)
        self.set_parameter("SIM_IE24_STATE", 8)
        self.wait_statustext("Status not running", timeout=40)
        self.try_arm(result=False,
                     expect_msg="Status not running")
        self.set_parameter("SIM_IE24_STATE", 2) # Explicitly set state to running

        # Test that error code does result in failsafe
        self.start_subtest("Protocol %i: Without taken off generator error should cause failsafe and disarm" % proto_ver)
        self.change_mode("STABILIZE")
        self.set_parameters({
            "DISARM_DELAY": 0,
            # a failsafe action of None does not disarm the vehicle on the ground
            "BATT%u_FS_LOW_ACT" % (elec_battery_instance + 1): 1,  # LAND
            "BATT%u_FS_CRT_ACT" % (elec_battery_instance + 1): 1,  # LAND
        })
        self.arm_vehicle()
        self.set_parameter("SIM_IE24_ERROR", 30)
        self.disarm_wait(timeout=1)
        self.set_parameter("SIM_IE24_ERROR", 0)
        self.set_parameter("DISARM_DELAY", 10)

    def test_IE24_low_capacity_failsafe(self, proto_ver):
        elec_battery_instance = 2
        fuel_battery_instance = 1
        self.set_parameters({
            "SERIAL5_PROTOCOL": 30,
            "SERIAL5_BAUD": 115200,
            "GEN_TYPE": 2,
            "BATT%u_MONITOR" % (fuel_battery_instance + 1): 18,  # fuel-based generator
            "BATT%u_MONITOR" % (elec_battery_instance + 1): 17,
            "SIM_IE24_ENABLE": proto_ver,
            "LOG_DISARMED": 1,
        })

        self.customise_SITL_commandline(["--serial5=sim:ie24"])

        self.change_mode('GUIDED')
        self.wait_ready_to_arm()

        original_fuel_m = self.wait_message_field_values('BATTERY_STATUS', {
            "charge_state": mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_OK
        }, instance=fuel_battery_instance)

        # set failsafe parameters
        self.set_parameters({
            "BATT%u_CAPACITY" % (fuel_battery_instance + 1): 100,
            "BATT%u_LOW_MAH" % (fuel_battery_instance + 1): original_fuel_m.battery_remaining - 5,
            "BATT%u_FS_LOW_ACT" % (fuel_battery_instance + 1): 1,  # LAND
        })

        self.takeoff(mode='GUIDED')
        self.wait_mode('LAND')
        self.wait_disarmed()
        self.set_rc(3, 1000)  # Restore the throttle stick since takeoff raised it.
