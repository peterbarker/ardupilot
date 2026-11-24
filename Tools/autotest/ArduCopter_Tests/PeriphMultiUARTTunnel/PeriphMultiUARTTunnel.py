'''
Copter autotest: PeriphMultiUARTTunnel

AP_FLAKE8_CLEAN
'''

import os
import pathlib

from pymavlink import mavutil

from pysim import util

from test_discovery import TestGroup
from test_discovery import testcase


class PeriphMultiUARTTunnel(TestGroup):
    @testcase(bucket="CopterTests2d")
    def PeriphMultiUARTTunnel(self):
        '''test peripheral multi-uart tunneling'''

        self.progress("Building Periph")
        periph_board = 'sitl_periph_can_to_serial'
        periph_builddir = util.reltopdir('build-periph')
        util.build_SITL(
            'bin/AP_Periph',
            board=periph_board,
            clean=False,
            configure=True,
            debug=True,
            extra_configure_args=[
                '--out', periph_builddir,
            ],
            # extra_defines={
            # },
        )

        binary_path = ""

        self.progress("Starting Periph simulation")
        self.context_push()
        periph_exp = None
        ex = None
        try:
            binary_path = pathlib.Path(periph_builddir, periph_board, 'bin', 'AP_Periph')
            periph_rundir = util.reltopdir('run-periph')
            if not os.path.exists(periph_rundir):
                os.mkdir(periph_rundir)
            periph_exp = util.start_SITL(
                binary_path,
                cwd=periph_rundir,
                stdout_prefix="periph",
                gdb=self.gdb,
                valgrind=self.valgrind,
                customisations=[
                    '-I', str(1),
                    '--serial0', 'mcast:',
                    '--serial1', 'tcp:2',
                    '--serial2', 'tcp:3',
                ],
                # AP_Periph is a supplementary program (no vehicle model):
                supplementary=True,
                speedup=self.speedup
            )
            self.expect_list_add(periph_exp)

            self.progress("Reconfiguring for multicast")
            self.customise_SITL_commandline([
                "--serial5=mcast:",
                # do not outrun the tunnel peripheral:
                "--sim-periph-lockstep",
            ],
                **self.callisto_sitl_kwargs()
            )

            self.set_parameters({
                'SERIAL5_PROTOCOL': 2,
                "CAN_P1_DRIVER": 1,  # needed for multicast state!
            })
            self.reboot_sitl()
            self.set_parameters({
                "CAN_D1_UC_SER_EN": 1, # enable serial
                "CAN_D1_UC_S1_IDX": 1,  # serial port number on CAN device
                "CAN_D1_UC_S1_NOD": 125,  # FIXME: set this explicitly
                "CAN_D1_UC_S1_PRO": 2,  # protocol to set on remote node

                "CAN_D1_UC_S2_IDX": 2,  # serial port number on CAN device
                "CAN_D1_UC_S2_NOD": 125,  # FIXME: set this explicitly
                "CAN_D1_UC_S2_PRO": 2,  # protocol to set on remote node
            })

            self.reboot_sitl()

            # uncomment this if you just want the test scenario set up for you:
            # self.delay_sim_time(100000)

            self.progress("Connect to the serial port on the peripheral, which should be talking mavlink")
            self.drain_mav()
            mav2 = mavutil.mavlink_connection(
                "tcp:localhost:5772",
                robust_parsing=True,
                source_system=9,
                source_component=9,
            )
            self.assert_receive_message("HEARTBEAT", mav=mav2, very_verbose=True, timeout=5)
            self.drain_mav()

            self.progress("Connect to the other serial port on the peripheral, which should also be talking mavlink")
            self.drain_mav()
            mav3 = mavutil.mavlink_connection(
                "tcp:localhost:5773",
                robust_parsing=True,
                source_system=10,
                source_component=10,
            )
            self.assert_receive_message("HEARTBEAT", mav=mav3, very_verbose=True, timeout=5)
            self.drain_mav()

            # make sure we continue to get heartbeats:
            tstart = self.get_sim_time()
            while True:
                now = self.get_sim_time()
                if now - tstart > 30:
                    break
                self.assert_receive_message('HEARTBEAT', mav=mav2, verbose=2)
                self.drain_mav()
                self.assert_receive_message('HEARTBEAT', mav=mav3, verbose=2)
                self.drain_mav()

        except Exception as e:  # noqa: BLE001
            self.print_exception_caught(e)
            ex = e
        finally:
            if periph_exp is not None:
                self.progress("Stopping Periph")
                self.expect_list_remove(periph_exp)
                util.pexpect_close(periph_exp)
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex
