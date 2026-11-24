'''
Copter autotest: EKF3SRCPerCore

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class EKF3SRCPerCore(TestGroup):
    @testcase(bucket="CopterTests2b")
    def EKF3SRCPerCore(self):
        '''Check SP/SH values in XKF4 for GPS (core 0) vs VICON (core 1) with targeted glitches.'''
        self.set_parameters({
            "EK3_ENABLE": 1,
            "AHRS_EKF_TYPE": 3,
            "VISO_TYPE": 2,
            "SERIAL5_PROTOCOL": 2,
            "EK3_SRC2_POSXY": 6,
            "EK3_SRC2_VELXY": 6,
            "EK3_SRC2_POSZ": 6,
            "EK3_SRC2_VELZ": 6,
            "EK3_SRC2_YAW": 6,
            "EK3_SRC_OPTIONS": 8  # bitmask: enable multiple EKF cores
        })

        self.customise_SITL_commandline(["--serial5=sim:vicon"])
        self.reboot_sitl()

        self.wait_ready_to_arm()
        self.takeoff(3)
        self.delay_sim_time(5, reason="EKF to stabilise after takeoff")

        self.progress("Injecting VICON glitch")
        self.set_parameter("SIM_VICON_GLIT_X", 100)
        self.set_parameter("SIM_VICON_GLIT_Y", 100)
        self.delay_sim_time(5, reason="VICON glitch to take effect")
        self.set_parameter("SIM_VICON_GLIT_X", 0)
        self.set_parameter("SIM_VICON_GLIT_Y", 0)

        self.progress("Injecting BARO glitch")
        self.set_parameter("SIM_BARO_GLITCH", 10)
        self.delay_sim_time(5, reason="baro glitch to take effect")
        self.set_parameter("SIM_BARO_GLITCH", 0)

        self.do_RTL()
        self.wait_disarmed(timeout=100)

        dfreader = self.dfreader_for_current_onboard_log()

        max_SP_core0 = 0
        max_SP_core1 = 0
        max_SH_core0 = 0
        max_SH_core1 = 0

        while True:
            m = dfreader.recv_match(type="XKF4")
            if m is None:
                break
            if not hasattr(m, "C"):
                continue
            if m.C == 0:
                max_SP_core0 = max(max_SP_core0, m.SP)
                max_SH_core0 = max(max_SH_core0, m.SH)
            elif m.C == 1:
                max_SP_core1 = max(max_SP_core1, m.SP)
                max_SH_core1 = max(max_SH_core1, m.SH)

        self.progress(f"Core 0 (GPS): SP={max_SP_core0:.2f}, SH={max_SH_core0:.2f}")
        self.progress(f"Core 1 (VICON): SP={max_SP_core1:.2f}, SH={max_SH_core1:.2f}")

        # Validate: horizontal glitch only on core 1, vertical glitch only on core 0
        if max_SP_core0 > 0.1:
            raise NotAchievedException("Unexpected high SP in core 0 (GPS) during VICON glitch")
        if max_SP_core1 < 0.9:
            raise NotAchievedException("VICON glitch did not raise SP in core 1")

        if max_SH_core1 > 0.5:
            raise NotAchievedException("Unexpected high SH in core 1 (VICON) during BARO glitch")
        if max_SH_core0 < 0.9:
            raise NotAchievedException("BARO glitch did not raise SH in core 0")
