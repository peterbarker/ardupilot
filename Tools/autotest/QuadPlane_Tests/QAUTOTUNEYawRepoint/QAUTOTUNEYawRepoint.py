'''
QuadPlane autotest: QAUTOTUNEYawRepoint

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class QAUTOTUNEYawRepoint(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def QAUTOTUNEYawRepoint(self):
        '''check QAutoTune survives a slow position-hold yaw re-point'''
        # beyond 5m of drift position hold commands a yaw re-point; the
        # low yaw acceleration here makes its traverse outlast the
        # failed-to-level timeout while staying under the rate-based
        # deferral threshold
        self.set_parameters({
            "Q_A_ACC_Y_MAX": 5,
            "Q_AUTOTUNE_AXES": 3,  # roll and pitch only
            # adjust tune so QAUTOTUNE can cope:
            "Q_AUTOTUNE_AGGR": 0.1,
            "Q_AUTOTUNE_MIN_D": 0.0004,
            "Q_A_RAT_RLL_P": 0.15,
            "Q_A_RAT_RLL_I": 0.25,
            "Q_A_RAT_RLL_D": 0.002,
            "Q_A_RAT_PIT_P": 0.15,
            "Q_A_RAT_PIT_I": 0.25,
            "Q_A_RAT_PIT_D": 0.002,
            "Q_A_RAT_YAW_P": 0.18,
            "Q_A_RAT_YAW_I": 0.018,
            "Q_A_ANG_RLL_P": 4.5,
            "Q_A_ANG_PIT_P": 4.5,
        })

        self.takeoff(15, mode='GUIDED')
        self.hover()
        self.change_mode("QLOITER")

        # record how far the vehicle drifts and yaws away from where
        # autotune starts, so the test fails if the re-point it exists
        # for stops happening, rather than passing without exercising it
        start_pos = self.assert_receive_message('GLOBAL_POSITION_INT')
        start_yaw = math.degrees(self.assert_receive_message('ATTITUDE').yaw)
        extremes = {"drift": 0.0, "yaw_change": 0.0}

        def record_extremes(mav, m):
            t = m.get_type()
            if t == 'GLOBAL_POSITION_INT':
                extremes["drift"] = max(extremes["drift"], self.get_distance_int(start_pos, m))
            elif t == 'ATTITUDE':
                change = abs((math.degrees(m.yaw) - start_yaw + 180) % 360 - 180)
                extremes["yaw_change"] = max(extremes["yaw_change"], change)

        self.context_push()
        self.install_message_hook_context(record_extremes)
        tstart = self.get_sim_time()
        self.context_collect('STATUSTEXT')
        self.change_mode("QAUTOTUNE")
        self.wait_text(
            "AutoTune: (Success|Failed to level).*",
            timeout=5000,
            check_context=True,
            regex=True,
        )
        self.context_pop()
        self.progress("drifted up to %.1fm and yawed up to %.0fdeg from the start" %
                      (extremes["drift"], extremes["yaw_change"]))
        if self.re_match.group(1) != "Success":
            raise NotAchievedException("autotune did not succeed")
        # position hold only re-points beyond 5m of drift, and then by
        # at least 90deg while tuning pitch (seen: 11.2m, 140deg)
        if extremes["drift"] < 5:
            raise NotAchievedException("only drifted %.1fm; no yaw re-point was exercised" %
                                       extremes["drift"])
        if extremes["yaw_change"] < 90:
            raise NotAchievedException("only yawed %.0fdeg; no large yaw re-point was exercised" %
                                       extremes["yaw_change"])
        self.progress("AUTOTUNE OK (%u seconds)" % (self.get_sim_time() - tstart))

        # leave QAUTOTUNE before disarming so the tuned gains are not saved
        self.change_mode("QLOITER")
        self.disarm_vehicle(force=True)
        self.reboot_sitl()  # may have drifted far from home
