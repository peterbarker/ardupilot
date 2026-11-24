'''
QuadPlane autotest: QAUTOTUNE

AP_FLAKE8_CLEAN
'''

from pymavlink.rotmat import Vector3

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class QAUTOTUNE(TestGroup):
    # first, on a fresh SITL: straight after PilotYaw it failed to level 5 times in 10
    # (PR #33628 fixes the underlying AutoTune level-timeout race)
    @testcase(bucket="QuadPlaneTests1c")
    def QAUTOTUNE(self):
        '''test Plane QAutoTune mode'''

        # adjust tune so QAUTOTUNE can cope
        self.set_parameters({
            "Q_AUTOTUNE_AGGR": 0.1,
            "Q_AUTOTUNE_MIN_D": 0.0004,
            "Q_A_RAT_RLL_P" : 0.15,
            "Q_A_RAT_RLL_I" : 0.25,
            "Q_A_RAT_RLL_D" : 0.002,
            "Q_A_RAT_PIT_P" : 0.15,
            "Q_A_RAT_PIT_I" : 0.25,
            "Q_A_RAT_PIT_D" : 0.002,
            "Q_A_RAT_YAW_P" : 0.18,
            "Q_A_RAT_YAW_I" : 0.018,
            "Q_A_ANG_RLL_P" : 4.5,
            "Q_A_ANG_PIT_P" : 4.5,
            })

        # this is a list of all parameters modified by QAUTOTUNE.  Set
        # them so that when the context is popped we get the original
        # values back:
        parameter_values = self.get_parameters([
            "Q_A_RAT_RLL_P",
            "Q_A_RAT_RLL_I",
            "Q_A_RAT_RLL_D",
            "Q_A_ANG_RLL_P",
            "Q_A_ACC_R_MAX",
            "Q_A_ACC_P_MAX",
            "Q_A_ACC_Y_MAX",
            "Q_A_RAT_PIT_P",
            "Q_A_RAT_PIT_I",
            "Q_A_RAT_PIT_D",
            "Q_A_ANG_PIT_P",
            "Q_A_RAT_YAW_P",
            "Q_A_RAT_YAW_I",
            "Q_A_RAT_YAW_FLTE",
            "Q_A_ANG_YAW_P",
        ])
        self.set_parameters(parameter_values)

        self.takeoff(15, mode='GUIDED')
        self.set_rc(3, 1500)
        self.change_mode("QLOITER")
        tstart = self.get_sim_time()
        self.context_collect('STATUSTEXT')
        self.change_mode("QAUTOTUNE")
        self.wait_text(
            "AutoTune: (Success|Failed to level).*",
            timeout=5000,
            check_context=True,
            regex=True,
        )
        if self.re_match.group(1) != "Success":
            raise NotAchievedException("autotune did not succeed")
        now = self.get_sim_time()
        self.progress("AUTOTUNE OK (%u seconds)" % (now - tstart))
        self.context_clear_collection('STATUSTEXT')

        self.progress("Landing to save gains")
        self.set_rc(3, 1200)
        self.wait_speed_vector(
            Vector3(float('nan'), float('nan'), 1.4),
            timeout=5,
        )
        self.wait_speed_vector(
            Vector3(0.0, 0.0, 0.0),
            timeout=20,
        )
        distance = self.distance_to_home()
        if distance > 20:
            raise NotAchievedException("wandered from home (distance=%f)" %
                                       (distance,))
        self.set_rc(3, 1000)
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 500:
                raise NotAchievedException("Did not get success message")
            self.send_mavlink_disarm_command()
            try:
                self.wait_text(
                    "AutoTune: Saved gains for Roll Pitch Yaw.*",
                    timeout=0.5,
                    check_context=True,
                    regex=True,
                    )
            except AutoTestTimeoutException:
                continue
            break

        self.wait_disarmed()
        self.reboot_sitl()  # far from home
