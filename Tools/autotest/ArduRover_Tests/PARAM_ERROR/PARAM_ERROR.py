'''
Rover autotest: PARAM_ERROR

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class PARAM_ERROR(TestGroup):
    @testcase(bucket="Rover")
    def PARAM_ERROR(self):
        '''test PARAM_ERROR mavlink message'''
        self.start_subtest("Non-existent parameter (get)")
        self.context_push()
        self.context_collect('PARAM_ERROR')
        self.install_messageprinter_handlers_context(['PARAM_ERROR'])
        self.send_get_parameter_direct("BOB")
        self.assert_received_message_field_values('PARAM_ERROR', {
            "target_system": 250,
            "target_component": 250,
            "param_id": 'BOB',
            "param_index": -1,
            "error": mavutil.mavlink.MAV_PARAM_ERROR_DOES_NOT_EXIST,
        }, check_context=True, very_verbose=True)
        self.context_pop()

        self.start_subtest("Non-existent parameter (get-by-id)")
        self.context_push()
        self.context_collect('PARAM_ERROR')
        self.install_messageprinter_handlers_context(['PARAM_ERROR'])
        non_existent_param_index = 32764  # hopefully, anyway....
        self.mav.mav.param_request_read_send(
            self.sysid_thismav(),
            1,
            bytes("", 'ascii'),
            non_existent_param_index
        )
        self.assert_received_message_field_values('PARAM_ERROR', {
            "target_system": 250,
            "target_component": 250,
            "param_id": '',
            "param_index": non_existent_param_index,
            "error": mavutil.mavlink.MAV_PARAM_ERROR_DOES_NOT_EXIST,
        }, check_context=True, very_verbose=True)
        self.context_pop()

        self.start_subtest("Non-existent parameter (set)")
        self.context_push()
        self.context_collect('PARAM_ERROR')
        self.send_set_parameter_direct("BOB", 1)
        self.assert_received_message_field_values('PARAM_ERROR', {
            "target_system": 250,
            "target_component": 250,
            "param_id": 'BOB',
            "param_index": -1,
            "error": mavutil.mavlink.MAV_PARAM_ERROR_DOES_NOT_EXIST,
        }, check_context=True, very_verbose=True)
        self.context_pop()

        self.start_subtest("Try to set a bad parameter value")
        self.context_push()
        self.context_collect('PARAM_ERROR')
        self.send_set_parameter_direct("SPRAY_ENABLE", float("nan"))
        self.assert_received_message_field_values('PARAM_ERROR', {
            "target_system": 250,
            "target_component": 250,
            "param_id": 'SPRAY_ENABLE',
            "param_index": -1,
            "error": mavutil.mavlink.MAV_PARAM_ERROR_VALUE_OUT_OF_RANGE,
        }, check_context=True, very_verbose=True)
        self.context_pop()

        self.start_subtest("Reject non-finite parameter values with current value")
        param_name = "SIM_ACC1_RND"
        current_value = self.get_parameter(param_name)
        for value in (float("nan"), float("inf"), float("-inf")):
            self.context_push()
            self.context_collect('PARAM_ERROR')
            self.send_set_parameter_direct(param_name, value)
            self.assert_received_message_field_values('PARAM_ERROR', {
                "target_system": 250,
                "target_component": 250,
                "param_id": param_name,
                "param_index": -1,
                "error": mavutil.mavlink.MAV_PARAM_ERROR_VALUE_OUT_OF_RANGE,
            }, check_context=True, very_verbose=True)
            reply = self.assert_receive_message(
                'PARAM_VALUE',
                condition="PARAM_VALUE.param_id == '%s'" % param_name,
            )
            self.assert_message_field_values(reply, {
                "param_value": current_value,
            })
            self.context_pop()
            self.assert_parameter_value(param_name, current_value)
