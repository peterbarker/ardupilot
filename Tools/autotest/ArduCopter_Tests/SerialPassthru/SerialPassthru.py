'''
Copter autotest: SerialPassthru

AP_FLAKE8_CLEAN
'''

import socket

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SerialPassthru(TestGroup):
    @testcase(bucket="CopterTests2c")
    def SerialPassthru(self):
        '''smoke test serial passthrough timeout and manual disable'''
        # SITL does not enforce UART thread ownership, so this cannot
        # reproduce the ChibiOS ownership failure.
        self.context_collect('STATUSTEXT')

        # test timeout recovery
        self.set_parameters({
            "SERIAL_PASSTIMO": 2,
            "SERIAL_PASS1": 0,
            "SERIAL_PASS2": 2,
        })
        self.wait_statustext("Passthru enabled", check_context=True, timeout=10, wallclock_timeout=True)
        self.wait_statustext("Passthru timed out", check_context=True, timeout=15, wallclock_timeout=True)
        # Verify MAVLink on port 0 responds after passthrough timeout
        self.assert_parameter_value("MAV_SYSID", 1)

        # Keep SERIAL0 available to disable passthrough after the ports lock.
        self.set_parameters({"SERIAL1_PROTOCOL": -1, "SERIAL2_PROTOCOL": -1})
        port1 = self.spare_network_port()
        port2 = self.spare_network_port(1)
        self.customise_SITL_commandline([
            "--serial1=tcp:%u" % port1,
            "--serial2=tcp:%u" % port2,
        ])
        self.context_clear_collection('STATUSTEXT')
        self.set_parameters({
            "SERIAL_PASSTIMO": 0,
            "SERIAL_PASS1": 2,
            "SERIAL_PASS2": 1,
        })
        self.wait_statustext("Passthru enabled", check_context=True, timeout=10, wallclock_timeout=True)
        self.delay_sim_time(2, reason="wait for passthrough ports to lock")
        with socket.create_connection(("127.0.0.1", port1), timeout=5) as serial1, \
                socket.create_connection(("127.0.0.1", port2), timeout=5) as serial2:
            for source, destination, data in ((serial1, serial2, b"serial1 to serial2"),
                                              (serial2, serial1, b"serial2 to serial1")):
                source.sendall(data)
                received = b''
                while len(received) < len(data):
                    chunk = destination.recv(len(data) - len(received))
                    if not chunk:
                        raise NotAchievedException("Passthrough socket closed")
                    received += chunk
                if received != data:
                    raise NotAchievedException("Passthrough data differs: %s" % received)
            self.set_parameter("SERIAL_PASS2", -1)
            self.wait_statustext("Passthru disabled", check_context=True, timeout=10, wallclock_timeout=True)
            self.delay_sim_time(1, reason="wait for passthrough forwarding to stop")
            for source, destination in ((serial1, serial2), (serial2, serial1)):
                destination.settimeout(0.5)
                source.sendall(b"after disable")
                try:
                    received = destination.recv(1)
                except socket.timeout:
                    continue
                if received:
                    raise NotAchievedException("Passthrough still forwards after disable")
        self.assert_parameter_value("MAV_SYSID", 1)
