'''
QuadPlane autotest: PlaneWindFailsafe

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneWindFailsafe(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def PlaneWindFailsafe(self):
        '''test the plane-wind-failsafe.lua example script'''
        self.install_example_script_context("plane-wind-failsafe.lua")
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SIM_WIND_DIR": 180,
        })
        self.reboot_sitl()

        self.takeoff(30, 'QLOITER')
        self.change_mode('LOITER')

        self.context_push()
        # EKF3 wind estimation is significantly less accurate than DCM
        # across the full wind speed range; use DCM for reliable estimates
        self.set_parameter('AHRS_EKF_TYPE', 1)

        self.start_subtest("No warning when wind is below warn threshold")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_parameter('SIM_WIND_SPD', 3)
        # The script runs every 1 simulated second; wait 30s to give it
        # enough iterations to detect any spurious warning
        self.delay_sim_time(30, reason="wind script to check threshold")
        if self.statustext_in_collections("Wind warning"):
            raise NotAchievedException("Got unexpected wind warning with wind=3m/s")
        if self.statustext_in_collections("Wind failsafe"):
            raise NotAchievedException("Got unexpected wind failsafe with wind=3m/s")
        self.context_pop()

        # Allow the EKF to converge at warning-level wind before exercising the
        # warning threshold; EKF wind estimation requires sustained fixed-wing flight
        self.set_parameter('SIM_WIND_SPD', 13)
        self.delay_sim_time(200, reason="EKF wind estimation to converge")

        self.start_subtest("Warning repeated approximately every 15 seconds, no failsafe")
        self.context_push()
        self.context_collect('STATUSTEXT')
        t_last = None
        for i in range(3):
            self.wait_statustext("Wind warning at", timeout=30)
            t_now = self.get_sim_time()
            if t_last is not None:
                interval = t_now - t_last
                if interval < 10 or interval > 25:
                    raise NotAchievedException(
                        "Warning interval %.1fs, expected ~15s" % interval
                    )
            t_last = t_now
        if self.statustext_in_collections("Wind failsafe"):
            raise NotAchievedException("Got failsafe while wind in warning-only range")
        self.context_pop()

        self.start_subtest("Failsafe and RTL when wind exceeds failsafe_speed")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_parameter('SIM_WIND_SPD', 25)
        self.wait_statustext("Wind failsafe at", check_context=True, timeout=120)
        self.wait_mode('RTL', timeout=10)
        self.context_pop()
        # switch to QRTL so the vehicle does a VTOL landing and disarms promptly
        self.change_mode('QRTL')

        self.start_subtest("Script is one-shot and stops after triggering failsafe")
        self.context_push()
        self.context_collect('STATUSTEXT')
        # SIM_WIND_SPD is 14 (restored by failsafe context_pop), in warning range;
        # wait more than one interval (15s) to confirm the script has stopped
        self.delay_sim_time(35, reason="wind script to confirm stopped")
        if self.statustext_in_collections("Wind warning"):
            raise NotAchievedException("Wind warning received after script should have exited")
        if self.statustext_in_collections("Wind failsafe"):
            raise NotAchievedException("Wind failsafe received after script should have exited")
        self.context_pop()

        self.context_pop()

        self.wait_altitude(-5, 1, relative=True, timeout=60)
        self.wait_disarmed(timeout=60)
        self.zero_throttle()
