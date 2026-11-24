'''
Rover autotest: ServoRelayEvents

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ServoRelayEvents(TestGroup):
    @testcase(bucket=["Rover", "BalanceBot"])
    def ServoRelayEvents(self):
        '''Test ServoRelayEvents'''
        for method in self.run_cmd, self.run_cmd_int:
            self.context_push()

            self.set_parameters({
                "RELAY1_FUNCTION": 1, # Enable relay 1 as a standard relay pin
                "RELAY2_FUNCTION": 1, # Enable relay 2 as a standard relay pin
            })
            self.reboot_sitl() # Needed for relay functions to take effect

            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=0, p2=0)
            off = self.get_parameter("SIM_PIN_MASK")
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=0, p2=1)
            on = self.get_parameter("SIM_PIN_MASK")
            if on == off:
                raise NotAchievedException(
                    "Pin mask unchanged after relay cmd")
            self.progress("Pin mask changed after relay command")
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=0, p2=0)

            self.set_message_rate_hz("RELAY_STATUS", 10)

            # default configuration for relays in sim have one relay:
            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 3,
                "on": 0,
            })
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=0, p2=1)
            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 3,
                "on": 1,
            })
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=1, p2=1)
            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 3,
                "on": 3,
            })
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=0, p2=0)
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=1, p2=0)
            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 3,
                "on": 0,
            })

            # add another relay and ensure that it changes the "present field"
            self.set_parameters({
                "RELAY6_FUNCTION": 1, # Enable relay 6 as a standard relay pin
                "RELAY6_PIN": 14, # Set pin number
            })
            self.reboot_sitl() # Needed for relay function to take effect
            self.set_message_rate_hz("RELAY_STATUS", 10) # Need to re-request the message since reboot

            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 35,
                "on": 0,
            })
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=5, p2=1)
            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 35,
                "on": 32,
            })
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=0, p2=1)
            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 35,
                "on": 33,
            })
            method(mavutil.mavlink.MAV_CMD_DO_SET_RELAY, p1=5, p2=0)
            self.assert_received_message_field_values('RELAY_STATUS', {
                "present": 35,
                "on": 1,
            })

            self.start_subtest("test MAV_CMD_DO_REPEAT_RELAY")
            self.context_push()
            self.context_set_speedup(1)
            method(
                mavutil.mavlink.MAV_CMD_DO_REPEAT_RELAY,
                p1=0,  # servo 1
                p2=5,  # 5 times
                p3=0.5,  # 1 second between being on
            )
            for value in 0, 1, 0, 1, 0, 1, 0, 1:
                self.wait_message_field_values('RELAY_STATUS', {
                    "on": value,
                })
            self.context_pop()
            self.delay_sim_time(3, reason="relay state to propagate")
            self.assert_received_message_field_values('RELAY_STATUS', {
                "on": 1,  # back to initial state
            })
            self.context_pop()

            self.start_subtest("test MAV_CMD_DO_SET_SERVO")
            for value in 1678, 2300, 0:
                method(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=13, p2=value)
                self.wait_servo_channel_value(13, value)

            self.start_subtest("test MAV_CMD_DO_REPEAT_SERVO")

            self.context_push()
            self.context_set_speedup(1)
            trim = self.get_parameter("SERVO13_TRIM")
            value = 2000
            method(
                mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO,
                p1=12,  # servo12
                p2=value,  # pwm
                p3=5,  # count
                p4=0.5,  # cycle time (1 second between high and high)
            )
            for value in trim, value, trim, value, trim, value, trim, value:
                self.wait_servo_channel_value(12, value)
            self.context_pop()

        self.set_message_rate_hz("RELAY_STATUS", 0)
