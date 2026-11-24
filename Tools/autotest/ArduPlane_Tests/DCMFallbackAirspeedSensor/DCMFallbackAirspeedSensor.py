'''
Plane autotest: DCMFallbackAirspeedSensor

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class DCMFallbackAirspeedSensor(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def DCMFallbackAirspeedSensor(self):
        '''when the AHRS falls back to DCM, DCM takes its airspeed from a
        healthy, in-use airspeed sensor even if the EKF3 it fell back
        from is rejecting that sensor: the EKF3 airspeed-rejection veto
        belongs to the EKF3 backend, not to DCM'''
        # CTUN.AsT values (AirspeedEstimateType):
        AIRSPEED_SENSOR = 1
        # NavFilterStatusBit values in XKF4.SS:
        REJECTING_AIRSPEED = 1 << 17
        DEAD_RECKONING = 1 << 18
        self.set_parameters({
            "AHRS_EKF_TYPE": 3,
            "EK3_IMU_MASK": 1,  # a single core, so XKF4 C=0 is the primary
            # stop the airspeed library itself disabling the sensor EKF3
            # rejects; this test is about the AHRS veto:
            "ARSPD_OPTIONS": 0,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.takeoff(50)
        self.change_mode('CIRCLE')

        AIRSPEED_SENSOR_UNHEALTHY = 1
        AIRSPEED_SENSOR_USING = 2

        def wait_ekf3_rejecting_airspeed():
            '''with the sensor healthy and in use, the AHRS stops
            reporting it as in use exactly when the EKF3 veto applies'''
            self.progress("Waiting for EKF3 to be rejecting the airspeed sensor")
            tstart = self.get_sim_time()
            rejecting_start = None
            while True:
                now = self.get_sim_time_cached()
                if now - tstart > 60:
                    raise NotAchievedException("EKF3 did not reject the airspeed sensor")
                a = self.assert_receive_message('AIRSPEED', instance=0)
                if a.flags & (AIRSPEED_SENSOR_UNHEALTHY | AIRSPEED_SENSOR_USING):
                    rejecting_start = None
                    continue
                if rejecting_start is None:
                    rejecting_start = now
                if now - rejecting_start >= 5:
                    return

        def vetoed_dcm_airspeed_types():
            '''return CTUN.AsT on the cycles logged so far where DCM is
            active and EKF3 is rejecting airspeed while not dead
            reckoning (the base veto condition)'''
            dfreader = self.dfreader_for_current_onboard_log()
            dcm_active = False
            ekf3_status = None
            ret = []
            while True:
                m = dfreader.recv_match(type=['MSG', 'XKF4', 'CTUN'])
                if m is None:
                    break
                mtype = m.get_type()
                if mtype == 'MSG':
                    if m.Message == "AHRS: DCM active":
                        dcm_active = True
                    elif m.Message.startswith("AHRS: ") and m.Message.endswith(" active"):
                        dcm_active = False
                    continue
                if mtype == 'XKF4':
                    if m.C == 0:
                        ekf3_status = m.SS
                    continue
                if not dcm_active or ekf3_status is None:
                    continue
                if (ekf3_status & REJECTING_AIRSPEED) and not (ekf3_status & DEAD_RECKONING):
                    ret.append(m.AsT)
            return ret

        # AP_AHRS only announces backend changes, so track the latest
        # one rather than searching for a matching statustext:
        active_backend = ["EKF3"]

        def record_active_backend(mav, m):
            if m.get_type() != 'STATUSTEXT':
                return
            if m.text.startswith("AHRS: ") and m.text.endswith(" active"):
                active_backend[0] = m.text[len("AHRS: "):-len(" active")]

        def wait_active_backend(name, minimum_duration=0, timeout=60):
            self.progress("Waiting for %s to be the active AHRS backend" % name)
            tstart = self.get_sim_time()
            pass_start = None
            while True:
                now = self.get_sim_time_cached()
                if now - tstart > timeout:
                    raise NotAchievedException(
                        "%s did not become the active AHRS backend (latest: %s)" %
                        (name, active_backend[0]))
                self.assert_receive_message('HEARTBEAT')
                if active_backend[0] != name:
                    pass_start = None
                    continue
                if pass_start is None:
                    pass_start = now
                if now - pass_start >= minimum_duration:
                    return

        self.context_push()
        self.install_message_hook_context(record_active_backend)
        try:
            self.start_subtest("Freeze the airspeed sensor low so EKF3 rejects it")
            # a low reading makes TECS add throttle rather than stall
            # the plane when DCM uses the sensor below:
            m = self.assert_receive_message('VFR_HUD')
            self.set_parameter("SIM_ARSPD_FAIL", max(m.airspeed - 8, 5))

            # EKF3 force-fuses airspeed once both its airspeed and
            # position have timed out, so after the GPS upset its
            # rejection comes and goes and may not overlap DCM being
            # active; retry the upset until the log shows they did:
            vetoed_types = []
            for attempt in range(3):
                wait_ekf3_rejecting_airspeed()
                self.start_subtest("Upset the EKF's GPS fusion to force a DCM fallback (attempt %u)" %
                                   (attempt+1))
                self.set_parameters({
                    "EK3_POS_I_GATE": 0,
                    "SIM_GPS1_HZ": 1,
                    "SIM_GPS1_LAG_MS": 1000,
                })
                wait_active_backend("DCM")
                self.delay_sim_time(20, reason="fly with DCM falling back")
                self.set_parameters({
                    "EK3_POS_I_GATE": 500,
                    "SIM_GPS1_HZ": 5,
                    "SIM_GPS1_LAG_MS": 100,
                })
                wait_active_backend("EKF3", minimum_duration=5)
                vetoed_types = vetoed_dcm_airspeed_types()
                self.progress("DCM-active CTUN samples with EKF3 rejecting airspeed: %u" %
                              len(vetoed_types))
                unexpected = set(vetoed_types) - {AIRSPEED_SENSOR}
                if unexpected:
                    raise NotAchievedException(
                        "DCM did not use the airspeed sensor EKF3 was rejecting (AsT values seen: %s)" %
                        sorted(set(vetoed_types)))
                if len(vetoed_types) >= 10:
                    break
            self.set_parameter("SIM_ARSPD_FAIL", 0)
        except Exception:
            self.disarm_vehicle(force=True)
            raise
        self.context_pop()
        self.disarm_vehicle(force=True)

        if len(vetoed_types) < 10:
            raise NotAchievedException(
                "Too few DCM-active samples with EKF3 rejecting airspeed (%u)" %
                len(vetoed_types))
