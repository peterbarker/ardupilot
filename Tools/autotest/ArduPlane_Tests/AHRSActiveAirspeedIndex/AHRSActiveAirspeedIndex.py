'''
Plane autotest: AHRSActiveAirspeedIndex

AP_FLAKE8_CLEAN
'''

import re

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AHRSActiveAirspeedIndex(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def AHRSActiveAirspeedIndex(self):
        '''every AHRS backend reports which airspeed sensor it is taking
        its airspeed from, and that follows ARSPD_PRIMARY.  Each backend
        fills in that index itself, and one which forgets is
        indistinguishable from one reporting the first sensor at the
        default ARSPD_PRIMARY, so the second sensor is made primary here'''
        # AIRSPEED.flags bit 1 is AIRSPEED_SENSOR_USING, set on the
        # sensor the AHRS is taking its airspeed from:
        AIRSPEED_SENSOR_USING = 2

        # bring up a VectorNav external AHRS on serial4 so that
        # AHRS_EKF_TYPE=11 (External) is a valid selection, and enable
        # EKF2 so that it too has a running backend:
        self.customise_SITL_commandline(["--serial4=sim:VectorNav"])
        self.set_parameters({
            "EAHRS_TYPE": 1,            # VectorNav
            "SERIAL4_PROTOCOL": 36,
            "SERIAL4_BAUD": 230400,
            "EK2_ENABLE": 1,
            "EK3_ENABLE": 1,
            "AHRS_EKF_TYPE": 3,
            "ARSPD2_TYPE": 2,
            "ARSPD2_USE": 1,
            "ARSPD2_PIN": 2,
        })
        self.context_collect("STATUSTEXT")
        self.reboot_sitl()

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

        def last_announced_backend():
            '''the backend named by the most recent "AHRS: <name> active"
            statustext, or None if none has been collected'''
            ret = None
            for m in self.context_collection("STATUSTEXT"):
                match = re.match(r"AHRS: (\S+) active", m.text)
                if match is not None:
                    ret = match.group(1)
            return ret

        # EKF3 is the boot selection, announced during boot; it comes
        # round again last so that it too is checked after a change of
        # backend, where only a fresh announcement will do:
        for (ahrs_type, shortname) in [
                (3, "EKF3"),
                (0, "DCM"),
                (2, "EKF2"),
                (10, "SIM"),
                (11, "External"),
                (3, "EKF3"),
        ]:
            self.start_subtest("%s (AHRS_EKF_TYPE=%u) reports the sensor it uses" %
                               (shortname, ahrs_type))
            self.set_parameter("AHRS_EKF_TYPE", ahrs_type)
            self.wait_statustext("AHRS: %s active" % shortname, timeout=60, check_context=True)
            # ARSPD_PRIMARY=1 selects the second sensor, 0 the first.
            # Start and end on the second sensor so a backend reporting
            # a constant zero is caught whether or not it has been
            # updated in the meantime:
            for primary in 1, 0, 1:
                self.set_parameter("ARSPD_PRIMARY", primary)
                wait_ahrs_using_airspeed_sensor(primary)
            # the flags must have come from the backend under test, not
            # from a fallback to DCM in the meantime:
            announced = last_announced_backend()
            if announced != shortname:
                raise NotAchievedException(
                    "Active backend is %s, expected %s" % (announced, shortname))
            # announcements from this stage are not evidence for the next:
            self.context_clear_collection("STATUSTEXT")
