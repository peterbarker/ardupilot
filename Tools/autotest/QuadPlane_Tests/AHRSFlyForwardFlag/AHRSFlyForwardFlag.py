'''
QuadPlane autotest: AHRSFlyForwardFlag

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AHRSFlyForwardFlag(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def AHRSFlyForwardFlag(self):
        '''ensure FlyForward flag is set appropriately'''
        self.set_parameters({
            "LOG_DISARMED": 1,
            "LOG_REPLAY": 1,
        })
        self.reboot_sitl()

        self.wait_mode('FBWA')  # initial mode from parameter
        self.delay_sim_time(10, reason="FBWA log entries")
        self.change_mode('QHOVER')
        self.delay_sim_time(10, reason="QHOVER log entries")

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(3, 2000)
        self.wait_altitude(20, 50, relative=True)
        self.context_collect('STATUSTEXT')
        self.change_mode('CRUISE')
        self.set_rc(3, 1500)
        self.wait_statustext('Transition started airspeed', check_context=True)
        self.wait_statustext('Transition airspeed reached', check_context=True)
        self.wait_statustext('Transition done', check_context=True)
        self.delay_sim_time(5, reason="transition data to be logged")
        self.change_mode('QHOVER')
        self.wait_airspeed(0, 5)
        self.delay_sim_time(5, reason="airspeed to drop")
        mlog_path = self.current_onboard_log_filepath()
        self.fly_home_land_and_disarm(timeout=600)

        mlog = self.dfreader_for_path(mlog_path)

        stage_require_fbwa = "require_fbwa"
        stage_wait_qhover = "wait_qhover"
        stage_verify_qhover_ff = "verify_qhover_ff"
        stage_wait_cruise = "wait_cruise"
        stage_cruise_wait_ff = "cruise_wait_ff"
        stage_qhover2 = "qhover2"
        stage_done = "done"
        stage = stage_require_fbwa
        msgs = {}
        seen_flag_set_in_cruise = False
        FF_BIT_MASK = (1 << 2)
        while stage != stage_done:
            m = mlog.recv_match()
            if m is None:
                raise NotAchievedException(f"Stuck in stage {stage}")
            m_type = m.get_type()
            msgs[m_type] = m

            if stage == stage_require_fbwa:
                if m_type == 'MODE':
                    if m.ModeNum == self.get_mode_from_mode_mapping('MANUAL'):
                        # manual to start with
                        continue
                    fbwa_num = self.get_mode_from_mode_mapping('FBWA')
                    print(f"{m.ModeNum=} {fbwa_num=}")
                    if m.ModeNum != fbwa_num:
                        raise ValueError(f"wanted mode={fbwa_num} got={m.ModeNum}")
                    continue
                if m_type == 'RFRN':
                    if not m.Flags & FF_BIT_MASK:
                        raise ValueError("Expected FF to be set in FBWA")
                    stage = stage_wait_qhover
                    continue
                continue

            if stage == stage_wait_qhover:
                if m_type == 'MODE':
                    qhover_num = self.get_mode_from_mode_mapping('QHOVER')
                    print(f"want={qhover_num} got={m.ModeNum}")
                    if m.ModeNum == qhover_num:
                        stage = stage_verify_qhover_ff
                        continue
                    continue
                continue

            if stage == stage_verify_qhover_ff:
                if m_type == 'RFRN':
                    if m.Flags & FF_BIT_MASK:
                        raise ValueError("Expected FF to be unset in QHOVER")
                    stage = stage_wait_cruise
                    continue
                continue

            if stage == stage_wait_cruise:
                if m_type == 'MODE':
                    want_num = self.get_mode_from_mode_mapping('CRUISE')
                    if m.ModeNum == want_num:
                        stage = stage_cruise_wait_ff
                        cruise_wait_ff_start = msgs['ATT'].TimeUS*1e-6
                        continue
                    continue
                continue

            if stage == stage_cruise_wait_ff:
                if m_type == 'MODE':
                    want_num = self.get_mode_from_mode_mapping('CRUISE')
                    if want_num != m.ModeNum:
                        if not seen_flag_set_in_cruise:
                            raise ValueError("Never saw FF get set")
                    if m.ModeNum == self.get_mode_from_mode_mapping('QHOVER'):
                        stage = stage_qhover2
                        continue
                    continue
                if m_type == 'RFRN':
                    flag_set = m.Flags & FF_BIT_MASK
                    now = msgs['ATT'].TimeUS*1e-6
                    delta_t = now - cruise_wait_ff_start
                    if delta_t < 8:
                        if flag_set:
                            raise ValueError("Should not see bit set")
                    if delta_t > 10:
                        if not flag_set and not seen_flag_set_in_cruise:
                            raise ValueError("Should see bit set")
                        seen_flag_set_in_cruise = True
                    continue
                continue

            if stage == stage_qhover2:
                '''bit should stay low for qhover 2'''
                if m_type == 'RFRN':
                    flag_set = m.Flags & FF_BIT_MASK
                    if flag_set:
                        raise ValueError("ff should be low in qhover")
                    continue
                if m_type == 'MODE':
                    if m.ModeNum != self.get_mode_from_mode_mapping('QHOVER'):
                        stage = stage_done
                        continue
                    continue
                continue

            raise NotAchievedException("Bad stage")
