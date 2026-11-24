'''
Rover autotest: Scripting

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class Scripting(TestGroup):
    def test_scripting_simple_loop(self):
        self.start_subtest("Scripting simple loop")

        self.context_push()

        messages = []

        def my_message_hook(mav, message):
            if message.get_type() != 'STATUSTEXT':
                return
            messages.append(message)

        self.install_message_hook_context(my_message_hook)

        self.set_parameter("SCR_ENABLE", 1)
        self.install_example_script_context("simple_loop.lua")
        self.reboot_sitl()
        self.delay_sim_time(10, reason="Lua script to initialise")

        self.context_pop()
        self.reboot_sitl()

        # check all messages to see if we got our message
        count = 0
        for m in messages:
            if "hello, world" in m.text:
                count += 1
        self.progress("Got %u hellos" % count)
        if count < 3:
            raise NotAchievedException("Expected at least three hellos")

    def test_scripting_internal_test(self):
        self.start_subtest("Scripting internal test")

        self.context_push()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_HEAP_SIZE": 1024000,
            "SCR_VM_I_COUNT": 1000000,
        })
        self.install_test_modules_context()
        self.install_mavlink_module_context()

        self.install_test_scripts_context([
            "scripting_test.lua",
            "scripting_require_test_2.lua",
            "math.lua",
            "strings.lua",
            "mavlink_test.lua",
        ])

        self.context_collect('STATUSTEXT')
        self.context_collect('NAMED_VALUE_FLOAT')

        self.reboot_sitl()

        for success_text in [
                "Internal tests passed",
                "Require test 2 passed",
                "Math tests passed",
                "String tests passed",
                "Received heartbeat from"
        ]:
            self.wait_statustext(success_text, check_context=True)

        for success_nvf in [
                "test",
        ]:
            self.assert_received_message_field_values("NAMED_VALUE_FLOAT", {
                "name": success_nvf,
            }, check_context=True)

        self.context_pop()
        self.reboot_sitl()

    def test_scripting_hello_world(self):
        self.start_subtest("Scripting hello world")

        self.context_push()
        self.context_collect("STATUSTEXT")
        self.set_parameter("SCR_ENABLE", 1)
        self.install_example_script_context("hello_world.lua")
        self.reboot_sitl()

        self.wait_statustext('hello, world', check_context=True, timeout=30)

        self.context_pop()
        self.reboot_sitl()

    def test_scripting_auxfunc(self):
        self.start_subtest("Scripting aufunc triggering")

        self.context_push()
        self.context_collect("STATUSTEXT")
        self.set_parameters({
            "SCR_ENABLE": 1,
            "RELAY1_FUNCTION": 1,
            "RELAY1_PIN": 1
        })
        self.install_example_script_context("RCIN_test.lua")
        self.reboot_sitl()

        self.wait_parameter_value("SIM_PIN_MASK", 121)
        self.wait_parameter_value("SIM_PIN_MASK", 123)
        self.wait_parameter_value("SIM_PIN_MASK", 121)

        self.context_pop()
        self.reboot_sitl()

    def test_scripting_print_home_and_origin(self):
        self.start_subtest("Scripting print home and origin")

        self.context_push()

        self.set_parameter("SCR_ENABLE", 1)
        self.install_example_script_context("ahrs-print-home-and-origin.lua")
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.wait_statustext("Home - ")
        self.wait_statustext("Origin - ")

        self.context_pop()
        self.reboot_sitl()

    def test_scripting_set_home_to_vehicle_location(self):
        self.start_subtest("Scripting set home to vehicle location")

        self.context_push()
        self.set_parameter("SCR_ENABLE", 1)
        self.install_example_script_context("ahrs-set-home-to-vehicle-location.lua")
        self.reboot_sitl()

        self.wait_statustext("Home position reset")

        self.context_pop()
        self.reboot_sitl()

    def test_scripting_serial_loopback(self):
        self.start_subtest("Scripting serial loopback test")

        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_SDEV_EN": 1,
            "SCR_SDEV1_PROTO": 28,
        })
        self.install_test_script_context("serial_loopback.lua")
        self.reboot_sitl()

        for success_text in [
                "driver -> device good",
                "device -> driver good",
        ]:
            self.wait_statustext(success_text, check_context=True)

        self.context_pop()
        self.reboot_sitl()

    def test_scripting_callback_time(self):
        self.start_subtest("Scripting callback time")

        self.context_collect('STATUSTEXT')
        self.set_parameter("SCR_ENABLE", 1)
        self.install_test_script_context("callback_time_test.lua")
        self.reboot_sitl()

        self.wait_statustext('Timing test passed', check_context=True, timeout=600)

    @testcase(bucket="Rover")
    def Scripting(self):
        '''Scripting test'''
        self.test_scripting_set_home_to_vehicle_location()
        self.test_scripting_print_home_and_origin()
        self.test_scripting_hello_world()
        self.test_scripting_simple_loop()
        self.test_scripting_internal_test()
        self.test_scripting_auxfunc()
        self.test_scripting_serial_loopback()
        self.test_scripting_callback_time()
