'''
Copter autotest: DeadReckoningInWind

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class DeadReckoningInWind(TestGroup):
    @testcase(bucket="CopterTests1e")
    def DeadReckoningInWind(self):
        '''ensure copter dead-reckoning on drag does not destabilise the EKF in wind'''
        # When GPS is lost in wind, the EKF dead-reckons the vehicle's
        # position using the drift produced by bluff-body drag
        # (EK3_DRAG_BCOEF_X/Y, EK3_DRAG_MCOEF).  Once GPS is gone the wind
        # states become unobservable, at which point the EKF freezes its
        # last airspeed estimate and synthesises an airspeed measurement
        # from it.  That synthetic airspeed is only valid for fly-forward
        # vehicles (it assumes zero sideslip); fusing it on a multicopter
        # corrupts the attitude and velocity states, which is dramatic
        # when the true airspeed no longer matches the frozen estimate
        # (e.g. the wind changes during the outage).  See issue #33451.
        self.context_push()
        self.set_parameters({
            # enable wind estimation and drag-based dead reckoning:
            "EK3_DRAG_BCOEF_X": 9.5,
            "EK3_DRAG_BCOEF_Y": 9.5,
            "EK3_DRAG_MCOEF": 0.082,
            # stop the dead-reckoning and EKF failsafes from changing
            # the flight mode so that we observe the raw EKF behaviour:
            "FS_DR_ENABLE": 0,
            "FS_EKF_ACTION": 0,
            # moderate wind from the North:
            "SIM_WIND_DIR": 0,
            "SIM_WIND_SPD": 5,
        })
        self.reboot_sitl()

        # take off in LOITER so that the helper leaves the throttle at
        # the hover point and the vehicle holds station in the wind:
        self.takeoff(50, mode='LOITER')
        # let the EKF learn the wind from drag while GPS is still available:
        self.delay_sim_time(60, reason="learn wind via drag fusion")

        self.progress("Disabling GPS to force dead-reckoning")
        self.set_parameter("SIM_GPS1_ENABLE", 0)
        # the wind drops away during the GPS outage.  The EKF froze its
        # last airspeed estimate when the wind became unobservable; with
        # the regression it keeps fusing that now-stale airspeed, which
        # corrupts the attitude/velocity estimate:
        self.set_parameter("SIM_WIND_SPD", 0)

        # While dead-reckoning the vehicle's *position* drifts (there is
        # no absolute position reference), so the reported position cannot
        # be used to detect a problem.  Instead we compare the EKF's
        # *attitude* estimate (ATTITUDE) against simulation truth
        # (SIMSTATE): with the fix the EKF stays stable and tracks attitude
        # to a couple of degrees (measured ~2deg), whereas fusing the
        # stale synthetic airspeed destabilises the estimate (measured
        # ~16deg and climbing).  An 8deg threshold separates the two.
        tstart = self.get_sim_time()
        max_err = 0
        while self.get_sim_time() - tstart < 90:
            sim = self.assert_receive_message('SIMSTATE')
            att = self.assert_receive_message('ATTITUDE')
            err = math.degrees(max(abs(att.roll - sim.roll), abs(att.pitch - sim.pitch)))
            max_err = max(max_err, err)
            if err > 8:
                raise NotAchievedException(
                    "EKF attitude diverged from truth by %.1f deg while dead-reckoning" % err)
        self.progress("Maximum EKF attitude error while dead-reckoning was %.1f deg" % max_err)

        self.disarm_vehicle(force=True)
        self.context_pop()
        self.reboot_sitl()
