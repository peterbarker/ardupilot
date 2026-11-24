'''
QuadPlane autotest: QRTLGradualAltDescent

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

import vehicle_test_suite

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class QRTLGradualAltDescent(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def QRTLGradualAltDescent(self):
        '''check gradual descent to RTL_ALTITUDE in QRTL'''
        qrtl_alt = 20
        rtl_altitude = 60
        self.set_parameters({
            "Q_RTL_ALT": qrtl_alt,
            "RTL_ALTITUDE": rtl_altitude,
            # decelerate gently, so the airbrake stage starts a long way
            # out.  That is where the approach altitude profile hands back
            # to the generic waypoint target, and any step in the target
            # altitude at that handover shows up clearly
            "Q_TRANS_DECEL": 0.6,
        })

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 100),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1500, 0, 100),
        ])

        self.wait_current_waypoint(2)

        # get well away from home, cruising at the mission altitude, so
        # QRTL's approach-phase ramp still has a long way to run
        self.wait_distance_to_home(900, 1000, timeout=180)

        entry_alt = self.get_altitude(relative=True)
        self.progress("Entering QRTL at altitude %.1fm" % entry_alt)
        self.change_mode('QRTL')

        # entry_alt (~100m) is well above RTL_ALTITUDE (60m).  Track the
        # altitude the fixed wing controller is actually chasing,
        # reconstructed from NAV_CONTROLLER_OUTPUT.alt_error, which is
        # (target - current) for as long as the fixed wing controller owns
        # altitude.  That covers the approach and the airbrake stage, up to
        # the handover to the VTOL position controller at QPOS_POSITION1,
        # after which alt_error means something else entirely.
        #
        # The target must ease down from entry_alt towards RTL_ALTITUDE
        # rather than snapping down while still ~900m short of home, must
        # not step at any point (in particular at the approach to airbrake
        # handover), and must actually make progress downwards - simply
        # holding entry_alt for ever is not a gradual descent.
        min_progress = 20           # m, target must come down at least this far

        class MonitorQRTLDescentRate(vehicle_test_suite.TestSuite.MessageHook):
            '''watches NAV_CONTROLLER_OUTPUT/GLOBAL_POSITION_INT and makes
            sure the QRTL approach altitude target eases down continuously,
            without stepping, towards RTL_ALTITUDE'''
            def __init__(self, suite, entry_alt, max_step_rate=5, max_step_fixed=2,
                         start_band=10):
                super(MonitorQRTLDescentRate, self).__init__(suite)
                # max_step_rate: m/s, target may not chase faster than this
                # max_step_fixed: m, allowed on top of max_step_rate * dt
                # start_band: m, how close to entry_alt the ramp's first
                # target must be for monitoring to start
                self.max_step_rate = max_step_rate
                self.max_step_fixed = max_step_fixed
                self.start_band = start_band
                self.entry_alt = entry_alt
                self.alt = entry_alt
                self.prev_target = None
                self.prev_t = None
                self.target = None
                self.started = False

            def process(self, mav, m):
                m_type = m.get_type()
                if m_type == 'GLOBAL_POSITION_INT':
                    self.alt = m.relative_alt * 0.001
                    return
                if m_type != 'NAV_CONTROLLER_OUTPUT':
                    return

                now = self.suite.get_sim_time_cached()
                self.target = self.alt + m.alt_error
                if not self.started:
                    # the first cycle after the mode change can still
                    # report alt_error against Q_RTL_ALT, before the
                    # approach ramp has a target of its own.  That is one
                    # sample of (entry_alt - Q_RTL_ALT), here 80m, and it
                    # is not the stepping this test looks for; start once
                    # the ramp owns the target.  Failing to start at all
                    # is caught after the hook is removed.
                    if abs(self.target - self.entry_alt) > self.start_band:
                        return
                    self.started = True
                if self.prev_target is not None:
                    dt = now - self.prev_t
                    step = abs(self.target - self.prev_target)
                    max_step = self.max_step_fixed + self.max_step_rate * dt
                    self.progress("QRTL descent: alt=%.1f target=%.1f step=%.1f" %
                                  (self.alt, self.target, step))
                    if step > max_step:
                        raise NotAchievedException(
                            "QRTL target altitude stepped by %.1fm in %.2fs "
                            "(max %.1fm), expected a continuous descent" %
                            (step, dt, max_step))
                self.prev_target = self.target
                self.prev_t = now

        self.context_push()
        monitor = MonitorQRTLDescentRate(self, entry_alt)
        self.install_message_hook_context(monitor)
        # the VTOL position controller taking over altitude marks the end
        # of the approach-phase ramp this test is checking
        self.wait_statustext('VTOL position1', timeout=300)
        self.context_pop()

        if monitor.target is None:
            raise NotAchievedException("Never saw a target altitude")
        if not monitor.started:
            raise NotAchievedException(
                "QRTL never set an approach target within %.1fm of the entry "
                "altitude (%.1fm); last target was %.1fm" %
                (monitor.start_band, entry_alt, monitor.target))
        if monitor.target > entry_alt - min_progress:
            raise NotAchievedException(
                "QRTL target altitude only came down from %.1fm to %.1fm, "
                "expected at least %.1fm of descent" %
                (entry_alt, monitor.target, min_progress))

        # let it continue home, transition and land normally
        self.wait_altitude(-5, 1, relative=True, timeout=240)
        self.wait_disarmed(timeout=60)
