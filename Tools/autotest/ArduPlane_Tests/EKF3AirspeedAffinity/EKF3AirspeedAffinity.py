'''
Plane autotest: EKF3AirspeedAffinity

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class EKF3AirspeedAffinity(TestGroup):
    def _EKF3AirspeedAffinity(self, force_dcm=False):
        '''shared body for EKF3AirspeedAffinity and
        EKF3AirspeedAffinityDCM; see those for what is asserted'''
        self.set_parameters({
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
            "EK3_AFFINITY": 8,  # airspeed affinity only
            "EK3_IMU_MASK": 3,  # two IMUs, so two cores
            "ARSPD2_TYPE": 2,
            "ARSPD2_USE": 1,
            "ARSPD2_PIN": 2,
            # the airspeed library's own failure handling would, with
            # EKF3 active, disable a sensor the EKF keeps rejecting
            # after a few seconds and move ARSPD_PRIMARY off it, at
            # which point both cores fall back to the same sensor and
            # there is no affinity left to test.  This test is about
            # the EKF's per-core behaviour, so turn that off:
            "ARSPD_OPTIONS": 0,
        })
        self.reboot_sitl()

        # AIRSPEED.flags bit 1 is AIRSPEED_SENSOR_USING, set on the
        # sensor the AHRS is taking its airspeed from; with affinity
        # that is the sensor selected by the EKF3 primary core.
        AIRSPEED_SENSOR_USING = 2

        def wait_airspeed_condition(description, condition, minimum_duration=0, timeout=30):
            '''wait for condition(sensor1_msg, sensor2_msg) to hold
            continuously for minimum_duration seconds of sim time,
            sampling a fresh AIRSPEED message from each sensor each
            time'''
            self.progress("Waiting for %s" % description)
            tstart = self.get_sim_time()
            pass_start = None
            while True:
                now = self.get_sim_time_cached()
                if now - tstart > timeout:
                    raise NotAchievedException("Did not get %s" % description)
                a0 = self.assert_receive_message('AIRSPEED', instance=0)
                a1 = self.assert_receive_message('AIRSPEED', instance=1)
                if not condition(a0, a1):
                    pass_start = None
                    continue
                if pass_start is None:
                    pass_start = now
                if now - pass_start >= minimum_duration:
                    return

        def wait_ahrs_using_airspeed_sensor(instance, minimum_duration=0):
            '''wait for the AHRS to be using airspeed sensor instance and
            not the other one, both holding for minimum_duration'''
            flags = [0, 0]
            flags[instance] = AIRSPEED_SENSOR_USING
            wait_airspeed_condition(
                "AHRS using airspeed sensor %u only" % (instance+1),
                lambda a0, a1: a0.flags == flags[0] and a1.flags == flags[1],
                minimum_duration=minimum_duration)

        def wait_airspeed_sensors_agree(minimum_duration):
            '''wait for the two airspeed sensors to read within 1m/s of
            each other for minimum_duration seconds'''
            wait_airspeed_condition(
                "airspeed sensors agreeing",
                lambda a0, a1: abs(a0.airspeed - a1.airspeed) <= 1,
                minimum_duration=minimum_duration,
                timeout=60)

        def fail_airspeed_sensor(instance, error=10):
            '''freeze airspeed sensor instance at error m/s above the
            reading of the other, healthy, sensor'''
            good = self.assert_receive_message('AIRSPEED', instance=1-instance)
            name = "SIM_ARSPD_FAIL" if instance == 0 else "SIM_ARSPD2_FAIL"
            self.set_parameter(name, good.airspeed + error)

        def restore_airspeed_sensor(instance):
            name = "SIM_ARSPD_FAIL" if instance == 0 else "SIM_ARSPD2_FAIL"
            self.set_parameter(name, 0)

        self.context_collect("STATUSTEXT")

        try:
            self.takeoff(alt=50)
            self.change_mode('CIRCLE')
            if force_dcm:
                # the AHRS reports the sensor its *active* backend
                # consumes.  EKF3 keeps running and lane-switching while
                # DCM is active, but DCM has no per-core selection, so the
                # AHRS must stay on ARSPD_PRIMARY (sensor 1) throughout:
                self.start_subtest("Force DCM as the active AHRS; EKF3 keeps running")
                self.set_parameter("AHRS_EKF_TYPE", 0)
                self.wait_statustext("DCM active", timeout=10, check_context=True)
            t_start = self.get_sim_time()

            self.start_subtest("Both sensors healthy: core 0 is primary and the AHRS uses sensor 1")
            wait_airspeed_sensors_agree(minimum_duration=5)
            wait_ahrs_using_airspeed_sensor(0)

            # (fault start, fault end, faulty core) for the log checks:
            fault_windows = []

            for (faulty, healthy) in [(0, 1), (1, 0)]:
                self.start_subtest("Fail airspeed sensor %u: lane moves to core %u" %
                                   (faulty+1, healthy))
                self.context_clear_collection("STATUSTEXT")
                t_fault = self.get_sim_time()
                fail_airspeed_sensor(faulty)
                # a 10m/s step is rejected outright, so the switch follows
                # within a few filter updates (observed 0.1-0.2s); 5s
                # leaves a wide margin while still catching a regression
                # to the slow error-score path
                self.wait_statustext("EKF3 lane switch %u" % healthy, timeout=5, check_context=True)
                self.context_clear_collection("STATUSTEXT")
                # with EKF3 active the AHRS follows the new primary core's
                # sensor; with DCM active it stays on ARSPD_PRIMARY.  Hold
                # the fault so the lane must stay with the healthy core and
                # the log has a decent window to check; longer than the 5s
                # the EKF takes to declare an airspeed timeout, so the
                # healthy core silently ceasing to fuse would show up in
                # the log:
                ahrs_sensor = 0 if force_dcm else healthy
                wait_ahrs_using_airspeed_sensor(ahrs_sensor, minimum_duration=8)
                t_restore = self.get_sim_time()
                restore_airspeed_sensor(faulty)
                fault_windows.append((t_fault, t_restore, faulty))

                self.start_subtest("Sensor %u restored: lane stays with core %u" % (faulty+1, healthy))
                # 15s of agreement comfortably exceeds the 10s a core must
                # have been off-primary before it can be selected again,
                # so a spurious switch back would be seen here:
                wait_airspeed_sensors_agree(minimum_duration=15)
                if self.statustext_in_collections("EKF3 lane switch"):
                    raise NotAchievedException("Unexpected lane switch after sensor restored")
                wait_ahrs_using_airspeed_sensor(ahrs_sensor)

            t_end = self.get_sim_time()
        except Exception:
            # get the vehicle on the ground so the framework can
            # reboot it during context cleanup:
            self.disarm_vehicle(force=True)
            raise
        self.disarm_vehicle(force=True)

        self.start_subtest("Check onboard log for per-core airspeed selection and fusion")
        dfreader = self.dfreader_for_current_onboard_log()
        # per-core count of XKFS samples seen, and the set of the
        # non-airspeed selection indexes each core used; these must be
        # constant and identical across cores as only airspeed affinity
        # is on:
        xkfs_count = {0: 0, 1: 0}
        other_selections = {0: set(), 1: set()}
        # per-window, per-core peak airspeed innovation (XKF3.IVT, m/s)
        # and count of samples failing the innovation gate (XKF4.SVT
        # is sqrt of the airspeed innovation test ratio):
        peak_innov = [{0: 0.0, 1: 0.0} for _ in fault_windows]
        rejected = [{0: 0, 1: 0} for _ in fault_windows]
        xkf4_count = [{0: 0, 1: 0} for _ in fault_windows]
        # XKF4.TS bit 4 is the airspeed timeout: 5s without a sample
        # passing the innovation gate
        timed_out = [{0: False, 1: False} for _ in fault_windows]
        while True:
            m = dfreader.recv_match(type=['XKFS', 'XKF3', 'XKF4'])
            if m is None:
                break
            t = m.TimeUS * 1e-6
            if t < t_start or t > t_end:
                continue
            if m.get_type() == 'XKFS':
                if m.AI != m.C:
                    raise NotAchievedException(
                        "Core %u selected airspeed sensor %u at t=%.1f, expected %u" %
                        (m.C, m.AI, t, m.C))
                xkfs_count[m.C] += 1
                other_selections[m.C].add((m.MI, m.BI, m.GI))
                continue
            for (i, (t_fault, t_restore, faulty)) in enumerate(fault_windows):
                if t < t_fault or t > t_restore:
                    continue
                if m.get_type() == 'XKF3':
                    peak_innov[i][m.C] = max(peak_innov[i][m.C], abs(m.IVT))
                    continue
                xkf4_count[i][m.C] += 1
                if m.SVT >= 1.0:
                    rejected[i][m.C] += 1
                if m.TS & (1 << 4):
                    timed_out[i][m.C] = True

        for core in (0, 1):
            if xkfs_count[core] == 0:
                raise NotAchievedException("No XKFS sensor-selection samples for core %u" % core)
            if len(other_selections[core]) != 1:
                raise NotAchievedException(
                    "Core %u changed compass/baro/GPS selection during the test (%s)" %
                    (core, other_selections[core]))
        if other_selections[0] != other_selections[1]:
            raise NotAchievedException(
                "Compass/baro/GPS selection differs between cores (%s vs %s)" %
                (other_selections[0], other_selections[1]))

        for (i, (t_fault, t_restore, faulty)) in enumerate(fault_windows):
            healthy = 1 - faulty
            self.progress("Fault window %u (sensor %u failed): "
                          "peak innovation core0=%.1f core1=%.1f rejected core0=%u/%u core1=%u/%u" %
                          (i, faulty+1, peak_innov[i][0], peak_innov[i][1],
                           rejected[i][0], xkf4_count[i][0], rejected[i][1], xkf4_count[i][1]))
            if peak_innov[i][faulty] < 5:
                raise NotAchievedException(
                    "Core %u did not see the fault on its sensor (peak innovation %.1f)" %
                    (faulty, peak_innov[i][faulty]))
            # the faulty core must keep reading and gating its own sensor
            # for the whole hold, not just see it once: XKF4 is logged
            # every filter update, so most samples must show rejection
            if rejected[i][faulty] < xkf4_count[i][faulty] / 2:
                raise NotAchievedException(
                    "Core %u rejected only %u of %u samples from its faulty sensor" %
                    (faulty, rejected[i][faulty], xkf4_count[i][faulty]))
            if xkf4_count[i][healthy] == 0:
                raise NotAchievedException("No XKF4 samples for healthy core %u in fault window" % healthy)
            if peak_innov[i][healthy] > 2.5:
                raise NotAchievedException(
                    "Core %u fusing the healthy sensor was disturbed (peak innovation %.1f)" %
                    (healthy, peak_innov[i][healthy]))
            if rejected[i][healthy] != 0:
                raise NotAchievedException(
                    "Core %u rejected %u healthy airspeed samples" %
                    (healthy, rejected[i][healthy]))
            if timed_out[i][healthy]:
                raise NotAchievedException(
                    "Core %u stopped fusing its healthy airspeed sensor" % healthy)

    @testcase(bucket="PlaneTests1b")
    def EKF3AirspeedAffinity(self):
        '''Test EKF3 airspeed affinity: each core fuses its own airspeed
        sensor, a fault on either sensor moves the primary lane to the
        core fusing the healthy sensor, the AHRS follows the primary
        core's selection, and the other affinity bits are unaffected'''
        self._EKF3AirspeedAffinity()

    @testcase(bucket="PlaneTests1b")
    def EKF3AirspeedAffinityDCM(self):
        '''EKF3AirspeedAffinity with DCM forced as the active AHRS after
        takeoff: EKF3 still lane-switches on its own sensors but the
        AHRS keeps reporting ARSPD_PRIMARY in use, as its active backend
        has no per-core selection'''
        self._EKF3AirspeedAffinity(force_dcm=True)
