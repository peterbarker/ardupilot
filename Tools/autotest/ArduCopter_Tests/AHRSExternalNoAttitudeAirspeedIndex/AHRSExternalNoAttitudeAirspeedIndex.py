'''
Copter autotest: AHRSExternalNoAttitudeAirspeedIndex

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AHRSExternalNoAttitudeAirspeedIndex(TestGroup):
    @testcase(bucket="CopterTests1a")
    def AHRSExternalNoAttitudeAirspeedIndex(self):
        '''the External AHRS backend reports which airspeed sensor is in
        use even when it has no attitude.  Unlike Plane, Copter does not
        fall back to DCM when the External backend has no attitude, so
        here the External backend's index is the one reported'''
        # AIRSPEED.flags bit 1 is AIRSPEED_SENSOR_USING, set on the
        # sensor the AHRS is taking its airspeed from:
        AIRSPEED_SENSOR_USING = 2

        # a VectorNav external AHRS is configured on serial4 but
        # nothing is attached, so it never supplies an attitude:
        self.set_parameters({
            "EAHRS_TYPE": 1,            # VectorNav
            "SERIAL4_PROTOCOL": 36,
            "SERIAL4_BAUD": 230400,
            "ARSPD_ENABLE": 1,
            "ARSPD_TYPE": 100,          # SITL
            "ARSPD_USE": 1,
            "ARSPD2_TYPE": 2,           # analog
            "ARSPD2_USE": 1,
            "ARSPD2_PIN": 2,
        })
        self.reboot_sitl()
        self.set_message_rate_hz('AIRSPEED', 10)

        # the change of backend is only announced if it happens after
        # boot, so select External now rather than before the reboot:
        self.context_collect("STATUSTEXT")
        self.set_parameter("AHRS_EKF_TYPE", 11)
        self.wait_statustext("AHRS: External active", timeout=60, check_context=True)

        # the External backend really has no attitude; the vehicle's
        # true attitude is never exactly zero on all three axes:
        m = self.assert_receive_message('ATTITUDE', verbose=True)
        if m.roll != 0 or m.pitch != 0 or m.yaw != 0:
            raise NotAchievedException("External AHRS unexpectedly has an attitude")

        def wait_ahrs_using_airspeed_sensor(instance, minimum_duration=2, timeout=30):
            '''wait for the AIRSPEED messages from both sensors to show
            the AHRS using sensor instance and not the other one,
            continuously for minimum_duration seconds'''
            description = "AHRS using airspeed sensor %u only" % (instance+1)
            self.progress("Waiting for %s" % description)
            flags = [0, 0]
            flags[instance] = AIRSPEED_SENSOR_USING
            tstart = self.get_sim_time()
            pass_start = None
            while True:
                now = self.get_sim_time_cached()
                if now - tstart > timeout:
                    raise NotAchievedException("Did not get %s" % description)
                a0 = self.assert_receive_message('AIRSPEED', instance=0)
                a1 = self.assert_receive_message('AIRSPEED', instance=1)
                if a0.flags != flags[0] or a1.flags != flags[1]:
                    pass_start = None
                    continue
                if pass_start is None:
                    pass_start = now
                if now - pass_start >= minimum_duration:
                    return

        # ARSPD_PRIMARY=1 selects the second sensor, 0 the first.
        # Start and end on the second sensor so a backend reporting a
        # constant zero is caught:
        for primary in 1, 0, 1:
            self.set_parameter("ARSPD_PRIMARY", primary)
            wait_ahrs_using_airspeed_sensor(primary)
