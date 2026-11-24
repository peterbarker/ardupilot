'''
Rover autotest: MAVProxy_SetModeUsingSwitch

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class MAVProxy_SetModeUsingSwitch(TestGroup):
    @testcase(bucket="Rover")
    def MAVProxy_SetModeUsingSwitch(self):
        """Set modes via mavproxy switch"""
        rcin_commandline_value = self.sitl_rcin_commandline_value(offset=1)
        rcin_endpoint = self.sitl_rcin_endpoint(offset=1)
        self.customise_SITL_commandline([
            "--rc-in-port", rcin_commandline_value,
        ])
        ex = None
        try:
            self.load_mission(self.arming_test_mission())
            self.wait_ready_to_arm()
            fnoo = [(1, 'MANUAL'),
                    (2, 'MANUAL'),
                    (3, 'RTL'),
                    (4, 'AUTO'),
                    (5, 'AUTO'),  # non-existent mode, should stay in RTL
                    (6, 'MANUAL')]
            mavproxy = self.start_mavproxy(sitl_rcin_port=rcin_endpoint)
            for (num, expected) in fnoo:
                mavproxy.send('switch %u\n' % num)
                self.wait_mode(expected)
            self.stop_mavproxy(mavproxy)
        except Exception as e:  # noqa: BLE001
            self.print_exception_caught(e)
            ex = e

        # if we don't put things back ourselves then the test cleanup
        # doesn't go well as we can't set the RC defaults correctly:
        self.customise_SITL_commandline([
        ])

        if ex is not None:
            raise ex
