'''
Rover autotest: ManyMAVLinkConnections

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class ManyMAVLinkConnections(TestGroup):
    @testcase(bucket="Rover")
    def ManyMAVLinkConnections(self):
        '''test testing >8 MAVLink connections'''
        self.set_parameters({
            "SERIAL3_PROTOCOL": 2,
            "SERIAL4_PROTOCOL": 2,
            "SERIAL5_PROTOCOL": 2,
            "SERIAL6_PROTOCOL": 2,
            "SERIAL7_PROTOCOL": 2,

            "NET_ENABLE": 1,

            "NET_P1_TYPE": 4,
            "NET_P1_IP0": 127,
            "NET_P1_IP1": 0,
            "NET_P1_IP2": 0,
            "NET_P1_IP3": 1,
            "NET_P1_PORT": 6700,
            "NET_P1_PROTOCOL": 2,

            "NET_P2_TYPE": 4,
            "NET_P2_IP0": 127,
            "NET_P2_IP1": 0,
            "NET_P2_IP2": 0,
            "NET_P2_IP3": 1,
            "NET_P2_PORT": 6701,
            "NET_P2_PROTOCOL": 2,

            "NET_P3_TYPE": 4,
            "NET_P3_IP0": 127,
            "NET_P3_IP1": 0,
            "NET_P3_IP2": 0,
            "NET_P3_IP3": 1,
            "NET_P3_PORT": 6702,
            "NET_P3_PROTOCOL": 2,

            "NET_P4_TYPE": 4,
            "NET_P4_IP0": 127,
            "NET_P4_IP1": 0,
            "NET_P4_IP2": 0,
            "NET_P4_IP3": 1,
            "NET_P4_PORT": 6703,
            "NET_P4_PROTOCOL": 2,

            "SCR_ENABLE": 1,
        })

        self.install_example_script_context("simple_loop.lua")
        self.customise_SITL_commandline([
            "--serial3=tcp:4",
            "--serial4=tcp:1",
        ])

        self.wait_statustext("hello, world")
        conns = {}
        endpoints = [
            "tcp:localhost:5761",
            self.sitl_serial_endpoint(1),
            self.sitl_serial_endpoint(2),
            "tcp:localhost:5764",
            self.sitl_serial_endpoint(5),
            self.sitl_serial_endpoint(6),
            self.sitl_serial_endpoint(7),
            "tcp:localhost:6700",
            "tcp:localhost:6701",
            "tcp:localhost:6702",
            "tcp:localhost:6703",
        ]
        for cstring in endpoints:
            self.progress(f"Connecting to {cstring}")
            c = mavutil.mavlink_connection(
                cstring,
                robust_parsing=True,
                source_system=7,
                source_component=7,
            )
            conns[cstring] = c

        for repeats in range(1, 5):
            for (endpoint, conn) in conns.items():
                while True:
                    self.progress(f"Checking endpoint {endpoint}")
                    m = self.assert_receive_message('STATUSTEXT', mav=conn, timeout=120)
                    self.drain_all_pexpects()
                    self.drain_mav()
                    if m.text == "hello, world":
                        self.progress(f"Received {m.text} from {endpoint}")
                        break
