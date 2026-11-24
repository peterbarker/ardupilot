'''
Plane autotest: SyntheticAirspeedNoSensor

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SyntheticAirspeedNoSensor(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def SyntheticAirspeedNoSensor(self):
        '''with no usable airspeed sensor and EKF3 active, the airspeed
        estimate is EKF3's own synthetic value (derived from its velocity
        and wind estimates), logged as EKF3_SYNTHETIC and not the DCM
        backend's DCM_SYNTHETIC, while remaining a plausible airspeed'''
        # CTUN.AsT values (AirspeedEstimateType):
        NO_NEW_ESTIMATE = 0
        DCM_SYNTHETIC = 2
        EKF3_SYNTHETIC = 3
        self.set_parameters({
            "ARSPD_USE": 0,        # do not consume the airspeed sensor
            "AHRS_EKF_TYPE": 3,
            "SIM_WIND_SPD": 6,
            "SIM_WIND_DIR": 45,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.takeoff(70)  # default sim wind ramps up to 60m
        self.change_mode('LOITER')

        # hold a plausible airspeed for a good window: this keeps the plane
        # flying and bounds the window for the log check below.  wait_airspeed()
        # reads VFR_HUD (the live pitot), so it does not itself prove the
        # synthetic value is used - the CTUN.AsT/As log check below does, and in
        # flight the speed scaler consumes the AHRS (synthetic) estimate:
        t_start = self.get_sim_time()
        self.wait_airspeed(8, 35, minimum_duration=20, timeout=120)
        t_end = self.get_sim_time()
        self.fly_home_land_and_disarm()

        # over that window CTUN.AsT must be EKF3's own synthetic (a
        # transient NO_NEW_ESTIMATE before the estimate settles is
        # tolerated).  It must never be DCM_SYNTHETIC: that would mean
        # EKF3 fell back to the DCM backend's estimate, and with the
        # sensor disabled it must never be AIRSPEED_SENSOR either.  The
        # AHRS may briefly fall back to DCM itself (e.g. on an EKF3 fault),
        # where DCM_SYNTHETIC is correct, so skip the DCM-active cycles:
        self.progress("Checking logged airspeed estimate type")
        dfreader = self.dfreader_for_current_onboard_log()
        seen = set()
        count = 0
        dcm_active = False
        dcm_count = 0
        synthetic_values = []
        while True:
            m = dfreader.recv_match(type=['MSG', 'CTUN'])
            if m is None:
                break
            if m.get_type() == 'MSG':
                if m.Message == "AHRS: DCM active":
                    dcm_active = True
                elif m.Message.startswith("AHRS: ") and m.Message.endswith(" active"):
                    dcm_active = False
                continue
            t = m.TimeUS * 1e-6
            if t < t_start or t > t_end:
                continue
            if dcm_active:
                dcm_count += 1
                continue
            seen.add(m.AsT)
            if m.AsT == EKF3_SYNTHETIC:
                synthetic_values.append(m.As)
            count += 1
        self.progress("Skipped %u DCM-active CTUN samples" % dcm_count)
        if count == 0:
            raise NotAchievedException("No CTUN messages found in the cruise window")
        if DCM_SYNTHETIC in seen:
            raise NotAchievedException(
                "EKF3 borrowed the DCM synthetic airspeed (saw AsT=DCM_SYNTHETIC)")
        if EKF3_SYNTHETIC not in seen:
            raise NotAchievedException(
                "EKF3 never produced its own synthetic airspeed (AsT values seen: %s)" %
                sorted(seen))
        unexpected = seen - {NO_NEW_ESTIMATE, EKF3_SYNTHETIC}
        if unexpected:
            raise NotAchievedException(
                "Unexpected airspeed estimate type(s) with the sensor disabled "
                "and EKF3 active: %s" % sorted(unexpected))
        # CTUN.As is the AHRS airspeed estimate (EAS).  VFR_HUD still reports the
        # live pitot even with ARSPD_USE=0, so check the logged estimate itself
        # is a plausible airspeed on the EKF3_SYNTHETIC cycles - i.e. the plane
        # really flew on EKF3's own synthetic value.  Use the median, which
        # ignores the brief overshoot while the wind estimate is still settling:
        median_synthetic = sorted(synthetic_values)[len(synthetic_values) // 2]
        if median_synthetic < 8 or median_synthetic > 45:
            raise NotAchievedException(
                "EKF3 synthetic airspeed implausible: median %.1f over %u samples (range [%.1f, %.1f])" %
                (median_synthetic, len(synthetic_values), min(synthetic_values), max(synthetic_values)))
