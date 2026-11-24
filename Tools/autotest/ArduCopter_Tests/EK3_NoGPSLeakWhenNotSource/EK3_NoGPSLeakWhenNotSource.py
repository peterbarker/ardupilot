'''
Copter autotest: EK3_NoGPSLeakWhenNotSource

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class EK3_NoGPSLeakWhenNotSource(TestGroup):
    @testcase(bucket="CopterTests1d")
    def EK3_NoGPSLeakWhenNotSource(self):
        '''verify EKF does not leak GPS position when GPS is not the configured source'''
        # With EKF configured to use optical flow (POSXY source is
        # not GPS) and GPS simultaneously healthy, verify the
        # reported vehicle position does not track a simulated GPS
        # glitch.  On master, NavEKF3_core::getGPSLLH() returns the
        # raw GPS fix irrespective of the configured POSXY source,
        # so getLLH() fallbacks leak the glitched GPS lat/lon to
        # callers (including GCS_MAVLINK::send_global_position_int
        # which ignores the return value of ahrs.get_location()).
        # This bypasses the EKF's source configuration and causes
        # vehicle code to follow GPS glitches/spoofing that the EKF
        # has correctly rejected.
        self.set_parameters({
            "EK3_SRC1_POSXY": 5,   # OPTFLOW (no optflow data provided)
            "EK3_SRC1_VELXY": 5,   # OPTFLOW
            "EK3_SRC1_POSZ": 1,    # BARO
            "EK3_SRC1_VELZ": 0,    # None
            "AHRS_EKF_TYPE": 3,
        })
        # this message is emitted as the vehicle comes up, so it can
        # arrive before a wait started afterwards; collect across the
        # reboot, which empties the collection as it goes
        self.context_collect('STATUSTEXT')

        self.reboot_sitl()

        # allow EKF to initialise: validOrigin set from GPS, filter
        # reaches steady AID_NONE state
        self.wait_statustext("EKF3 IMU0 initialised", timeout=30, check_context=True)

        # capture baseline reported position
        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        baseline_lat = m.lat
        baseline_lon = m.lon
        self.progress("Baseline: lat=%d lon=%d" % (baseline_lat, baseline_lon))

        # glitch simulated GPS by ~0.005 deg latitude (~555 m north)
        self.set_parameter("SIM_GPS1_GLTCH_X", 0.005)

        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        lat_change_deg = abs(m.lat - baseline_lat) * 1e-7
        self.progress("After GPS glitch: lat=%d lon=%d (baseline lat=%d, change=%.6f deg)" %
                      (m.lat, m.lon, baseline_lat, lat_change_deg))

        if lat_change_deg > 0.001:
            raise NotAchievedException(
                "GPS position leaked into reported location despite "
                "EK3_SRC1_POSXY=OPTFLOW: baseline lat %d, after glitch %d "
                "(delta %.6f deg)" %
                (baseline_lat, m.lat, lat_change_deg))
