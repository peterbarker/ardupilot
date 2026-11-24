'''
Copter autotest: LuaParamLockdown

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class LuaParamLockdown(TestGroup):
    @testcase(bucket="CopterTests2c")
    def LuaParamLockdown(self):
        '''test param-lockdown.lua applet'''
        self.set_parameters({
            'SCR_ENABLE': 1,
        })

        self.context_push()

        self.install_mavlink_module_context("MAVLink")
        self.install_applet_script_context("param-lockdown.lua")
        self.reboot_sitl()

        # collect original parameter values for everything other than
        # SCR_ENABLE and PARAM_LOCK_ENAB.  This resolves an issue at
        # the end of the test where the popped context wants to set
        # PARAM_LOCK_ENAB back to its original value (enabled!),
        # which can cause problems resetting
        orig_parameter_values = self.get_parameters([
            'RTL_ALT_M',
            'DISARM_DELAY',
        ])

        self.wait_ready_to_arm()  # scripts will be ready by now!
        self.start_subtest("set RTL_ALT_M freely")
        self.context_collect("STATUSTEXT")
        self.set_parameter("RTL_ALT_M", 0.23)
        self.set_parameter("RTL_ALT_M", 0.28)
        # Ensure there are no scripting errors.
        error = self.statustext_in_collections("Internal Error")
        if error is not None:
            raise NotAchievedException("Script errored out in positive test.")
        self.context_stop_collecting("STATUSTEXT")

        self.start_subtest("Unable to set DISARM_DELAY freely")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.context_collect('PARAM_ERROR')
        old_disarm_delay_value = self.get_parameter('DISARM_DELAY')
        self.send_set_parameter_direct('DISARM_DELAY', 78)
        self.wait_statustext('param-lockdown: param set denied (DISARM_DELAY)', check_context=True)
        self.assert_received_message_field_values('PARAM_ERROR', {
            "target_system": 250,
            "target_component": 250,
            "param_id": 'DISARM_DELAY',
            "param_index": -1,
            "error": mavutil.mavlink.MAV_PARAM_ERROR_PERMISSION_DENIED,
        }, check_context=True, very_verbose=True)
        self.assert_parameter_value('DISARM_DELAY', old_disarm_delay_value)
        self.context_pop()

        original_source = self.mav.mav.srcSystem
        try:
            for source in (255, 256, 70000, 0x80000000, 0xFFFFFFFF):
                self.start_subtest("PARAM_ERROR reply to source %u" % source)
                self.context_push()
                self.context_collect('PARAM_ERROR')
                self.mav.mav.srcSystem = source
                self.send_set_parameter_direct('DISARM_DELAY', 78)
                reply = self.assert_received_message_field_values('PARAM_ERROR', {
                    "target_system": source,
                    "target_component": 250,
                    "param_id": 'DISARM_DELAY',
                    "param_index": -1,
                    "error": mavutil.mavlink.MAV_PARAM_ERROR_PERMISSION_DENIED,
                }, check_context=True)
                if bool(reply.get_header().incompat_flags & mavutil.mavlink.MAVLINK_IFLAG_TARGET32) != (source > 255):
                    raise NotAchievedException("Incorrect PARAM_ERROR target header")
                self.assert_parameter_value('DISARM_DELAY', old_disarm_delay_value)
                self.context_pop()
        finally:
            self.mav.mav.srcSystem = original_source

        self.start_subtest("Disabling applet via parameter should allow freely setting DISARM_DELAY")
        self.set_parameter("PARAM_LOCK_ENAB", 0)
        self.set_parameter("DISARM_DELAY", 56)

        self.start_subtest("Re-enabling applet via parameter should stop freely setting DISARM_DELAY")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.context_collect('PARAM_ERROR')
        self.set_parameter("PARAM_LOCK_ENAB", 1)
        old_disarm_delay_value = self.get_parameter('DISARM_DELAY')
        self.send_set_parameter_direct('DISARM_DELAY', 78)
        self.wait_statustext('param-lockdown: param set denied (DISARM_DELAY)', check_context=True)
        self.assert_received_message_field_values('PARAM_ERROR', {
            "target_system": 250,
            "target_component": 250,
            "param_id": 'DISARM_DELAY',
            "param_index": -1,
            "error": mavutil.mavlink.MAV_PARAM_ERROR_PERMISSION_DENIED,
        }, check_context=True, very_verbose=True)
        self.assert_parameter_value('DISARM_DELAY', old_disarm_delay_value)
        self.context_pop()

        self.start_subtest("Ensure that parameter values are persistent")
        self.set_parameter('DISARM_DELAY', 111)
        self.reboot_sitl()
        self.assert_parameter_value('DISARM_DELAY', 111)

        # set parameters to their original value so the following
        # context pop should only need to set PARAM_LOCK_ENAB back to
        # its default value of 1.  That prevents permission errors
        # created by param-lockdown.lua!
        self.set_parameters(orig_parameter_values)

        self.context_pop()
