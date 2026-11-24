'''
Plane autotest: PPPPeriph

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PPPPeriph(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def PPPPeriph(self):
        '''verify PPP-over-TCP link to an AP_Periph (sitl_periph_PPP) companion'''
        self.context_collect('STATUSTEXT')

        # Pre-build the plane with PPP + networking-tests, build the matching
        # AP_Periph (sitl_periph_PPP), spawn the periph child and restart
        # SITL with SERIAL5 wired to the {port} the helper allocates. The
        # plane binary is auto-restored and the periph terminated on
        # implicit context pop.
        #
        # CAN_P1_DRIVER=1 is needed so plane publishes the SITL multicast
        # sim state the periph blocks on at boot.
        self.set_parameters({
            "NET_ENABLE": 1,
            "SERIAL5_PROTOCOL": 48,
            "CAN_P1_DRIVER": 1,
        })
        self.restart_SITL_frame(
            'quadplane-PPP',
            # lockstep: the periph's PPP endpoint must not fall behind
            # simulation time when the runner is loaded
            customisations=['--serial5=tcp:{port}', '--sim-periph-lockstep'],
        )

        # Plane should announce PPP backend init.  Time this on the wall
        # clock: bringing the link up runs pppd, connects a TCP socket to
        # the AP_Periph child and negotiates LCP/IPCP, none of which is
        # paced by the simulation.  Budgeted in simulated time this
        # allowed 0.9s of real time for all of that before giving up:
        #     PPPPeriph ... Failed to receive text: ppp[0]: started
        # with the plane having booted cleanly right up to "ArduPilot
        # Ready" in the meantime.
        self.wait_statustext("PPP[0]: started", check_context=True, timeout=30,
                             wallclock_timeout=True)

        # Pre-IPCP, the first NET: IP message is the static fallback
        # (192.168.x.x). Wait for an IPCP-assigned address - matched as any
        # 10.x to decouple this test from the periph param file's exact
        # numbering but distinct enough to ignore the fallback. The periph
        # cannot complete IPCP unless its own PPP stack is alive, so seeing
        # an assigned address on the plane implicitly proves both ends of
        # the link are running.
        m = self.wait_statustext(r"NET: IP\s+10\.\d+\.\d+\.\d+",
                                 check_context=True, regex=True, timeout=30,
                                 wallclock_timeout=True)
        self.progress("PPP link established: %s" % m.text.strip())
