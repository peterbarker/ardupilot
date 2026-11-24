'''
Copter autotest: CompassMot

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CompassMot(TestGroup):
    @testcase(bucket="CopterTests2b")
    def CompassMot(self):
        '''test code that adjust mag field for motor interference'''
        # simulate real motor interference for the calibration to
        # learn: SIM_MAG_MOT is applied as mGauss per amp of battery
        # current.  Without it the calibration fits noise and whatever
        # attitude changes the (unanchored, throttled-up) vehicle's
        # excursions produce - the historical strictly-positive
        # compensation check passed or failed by accident.  Negative
        # interference so the learned compensation is positive:
        self.set_parameters({
            "SIM_MAG_MOT_X": -10,
            "SIM_MAG_MOT_Y": -10,
            "SIM_MAG_MOT_Z": -10,
            "SIM_CLAMP_CH": 11,
        })
        # the calibration below makes the *firmware* write its results,
        # so the suite has no record of them and they would survive into
        # every test which follows in this session.  Register them for
        # restoration on context_pop():
        self.context_preserve_parameters([
            "COMPASS_MOTCT",
            "COMPASS_MOT_X", "COMPASS_MOT_Y", "COMPASS_MOT_Z",
            "COMPASS_MOT2_X", "COMPASS_MOT2_Y", "COMPASS_MOT2_Z",
            "COMPASS_MOT3_X", "COMPASS_MOT3_Y", "COMPASS_MOT3_Z",
        ])

        # hold the vehicle in the simulated clamp so the calibration
        # is bench-static, as compassmot is in the real world - at
        # full throttle an unclamped SITL vehicle takes off and
        # crashes mid-calibration:
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=2000)
        self.wait_statustext("SITL: Clamp: grabbed vehicle")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            0,  # p1
            0,  # p2
            0,  # p3
            0,  # p4
            0,  # p5
            1,  # p6
            0  # p7
        )
        self.context_collect("STATUSTEXT")
        self.wait_statustext("Starting calibration", check_context=True)
        self.wait_statustext("Current", check_context=True)
        rc3_min = self.get_parameter('RC3_MIN')
        rc3_max = self.get_parameter('RC3_MAX')
        rc3_dz = self.get_parameter('RC3_DZ')

        def set_rc3_for_throttle_pct(thr_pct):
            value = int((rc3_min+rc3_dz) + (thr_pct/100.0) * (rc3_max-(rc3_min+rc3_dz)))
            self.progress("Setting rc3 to %u" % value)
            self.set_rc(3, value)

        throttle_in_pct = 0
        set_rc3_for_throttle_pct(throttle_in_pct)
        self.assert_received_message_field_values("COMPASSMOT_STATUS", {
            "interference": 0,
            "throttle": throttle_in_pct
        }, verbose=True, very_verbose=True)
        tstart = self.get_sim_time()
        delta = 5
        while True:
            if self.get_sim_time_cached() - tstart > 60:
                raise NotAchievedException("did not run through entire range")
            throttle_in_pct += delta
            self.progress("Using throttle %f%%" % throttle_in_pct)
            set_rc3_for_throttle_pct(throttle_in_pct)
            self.wait_message_field_values("COMPASSMOT_STATUS", {
                "throttle": throttle_in_pct * 10.0,
            }, verbose=True, very_verbose=True, epsilon=1)
            if throttle_in_pct == 0:
                # finished counting down
                break
            if throttle_in_pct == 100:
                # start counting down
                delta = -delta

        m = self.wait_message_field_values("COMPASSMOT_STATUS", {
            "throttle": 0,
        }, verbose=True)
        # the calibration should recover the injected interference:
        # compensation is the negation of SIM_MAG_MOT (measured
        # recovery error ~0.001%; tolerance is generous)
        for axis in "X", "Y", "Z":
            fieldname = "Compensation" + axis
            value = getattr(m, fieldname)
            if abs(value - 10.0) > 0.5:
                raise NotAchievedException(
                    "%s %f does not match injected interference (want 10.0)" %
                    (fieldname, value))

        # it's kind of crap - but any command-ack will stop the
        # calibration
        self.mav.mav.command_ack_send(0, 1)
        self.wait_statustext("Calibration successful")

        # we don't anchor the vehicle when we do this test.  If we
        # don't reboot then we end up smacking into the ground in the
        # next test...
        self.reboot_sitl()
