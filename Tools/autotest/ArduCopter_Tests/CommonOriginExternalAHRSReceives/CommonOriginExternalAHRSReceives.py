'''
Copter autotest: CommonOriginExternalAHRSReceives

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CommonOriginExternalAHRSReceives(TestGroup):
    @testcase(bucket="CopterTests1a")
    def CommonOriginExternalAHRSReceives(self):
        '''ensure an EKF-established origin is shared into the ExternalAHRS'''
        # SensAItion in legacy IMU-only mode is a healthy ExternalAHRS which
        # never establishes an origin of its own: it emits only IMU packets,
        # never the INS packet that would set one.  EKF3 obtains the origin
        # from the SITL GPS, and the common-origin code must then share that
        # origin into the ExternalAHRS.  With AHRS_EKF_TYPE=11 the ExternalAHRS
        # pre-arm check requires an origin, so a failure to share it surfaces
        # as the "ExternalAHRS: No origin" pre-arm message.
        self.customise_SITL_commandline(["--serial4=sim:SensAItion"])
        self.set_parameters({
            'EAHRS_TYPE': 11,        # SensAItion
            'EAHRS_SENSORS': 14,     # IMU|BARO|COMPASS (no GPS)
            'EAHRS_OPTIONS': 0,      # legacy IMU-only mode (never sets own origin)
            'SERIAL4_PROTOCOL': 36,
            'SERIAL4_BAUD': 460800,
            'GPS1_TYPE': 1,          # SITL GPS feeds the EKFs
            'EK3_ENABLE': 1,
            'AHRS_EKF_TYPE': 11,     # configured EXTERNAL: origin pre-arm check active
            'INS_GYR_CAL': 1,
        })
        # collect before the reboot: the messages waited for below are
        # emitted as the vehicle comes up, so a collection started
        # afterwards is created too late to catch them.  reboot_sitl()
        # empties collections as it goes, so the identical messages
        # from before the reboot cannot satisfy those waits.
        self.context_collect('STATUSTEXT')

        self.reboot_sitl()

        # EKF3 obtains an origin from the SITL GPS:
        self.wait_statustext("EKF3 IMU0 origin set", timeout=60, check_context=True)
        self.delay_sim_time(2, reason="origin to be shared into the ExternalAHRS")

        # The ExternalAHRS must now hold that origin.  Run the pre-arm checks
        # and confirm the "No origin" failure does not appear (other failures
        # are expected as the IMU-only ExternalAHRS supplies no position).
        saw_prearm = False
        tstart = self.get_sim_time()
        prearm_last_send = 0
        while self.get_sim_time_cached() - tstart < 20:
            now = self.get_sim_time_cached()
            if now - prearm_last_send > 1:
                prearm_last_send = now
                self.send_mavlink_run_prearms_command()
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if m is None:
                continue
            if "ExternalAHRS: No origin" in m.text:
                raise NotAchievedException("ExternalAHRS did not receive the EKF3 origin")
            if "PreArm" in m.text:
                saw_prearm = True
        if not saw_prearm:
            raise NotAchievedException("pre-arm checks did not run")
