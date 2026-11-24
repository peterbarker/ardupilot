'''
QuadPlane autotest: CruiseRecovery

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CruiseRecovery(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def CruiseRecovery(self):
        '''test QAssist recovery in CRUISE mode from bad attitude'''
        self.install_example_script_context("sim_arming_pos.lua")
        self.install_terrain_handlers_context()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "AHRS_EKF_TYPE": 10,
            "EK3_ENABLE": 0,
            "LOG_DISARMED": 1,
            "Q_LAND_FINAL_SPD" : 2,
            "HOME_RESET_ALT" : -1,
            # disable simulated battery voltage sag so the repeated max-thrust
            # recoveries are not slowed by it
            "SIM_BATT_RES_OHM" : 0,
        })

        self.reboot_sitl(check_position=True)

        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "SIM_APOS_ENABLE" : 1,
            "SIM_APOS_PIT" : -70,
            "SIM_APOS_POS_D" : -200,
            "SIM_APOS_POS_E" : 400,
            "SIM_APOS_POS_N" : 200,
            "SIM_APOS_RLL" : 150,
            "SIM_APOS_VEL_X" : 40.0,
            "SIM_APOS_VEL_Y" : 0.0,
            "SIM_APOS_VEL_Z" : 0.0,
            "SIM_APOS_YAW" : 250,
            "SIM_APOS_GX" : 0,
            "SIM_APOS_GY" : 0,
            "SIM_APOS_GZ" : 0,
            "SIM_APOS_MODE" : 7, # CRUISE
            })

        self.scripting_restart()
        self.wait_text("Loaded arm pose", check_context=True)
        self.wait_ready_to_arm()

        # set cruise target airspeed
        self.set_rc(3, 1500)

        target_airspeed = self.get_parameter("AIRSPEED_CRUISE")

        NTESTS = 20
        for t in range(NTESTS):
            self.change_mode("FBWA")
            self.delay_sim_time(3, reason="mode change to stabilise")
            self.progress("Fast CRUISE Recovery test %u" % t)
            self.arm_vehicle(force=True)
            self.delay_sim_time(3, reason="recovery to begin")
            # reset target alt and heading using stick inputs
            self.set_rc(2, 1600)
            self.set_rc(2, 1500)
            self.set_rc(1, 1600)
            self.set_rc(1, 1500)
            self.wait_attitude(desroll=0, despitch=0, timeout=10, tolerance=10)
            self.wait_airspeed(target_airspeed-1, target_airspeed+1)
            final_alt = self.assert_receive_message('TERRAIN_REPORT').current_height
            if final_alt < 100:
                raise NotAchievedException(f"Final alt {final_alt:.1f}")

            self.disarm_vehicle(force=True)

        self.progress("Setup for inverted slow recovery")
        self.set_parameters({
            "SIM_APOS_ENABLE" : 1,
            "SIM_APOS_PIT" : 0,
            "SIM_APOS_POS_D" : -200,
            "SIM_APOS_POS_E" : 400,
            "SIM_APOS_POS_N" : 200,
            "SIM_APOS_RLL" : 180,
            "SIM_APOS_VEL_X" : 0.0,
            "SIM_APOS_VEL_Y" : 0.0,
            "SIM_APOS_VEL_Z" : 0.0,
            "SIM_APOS_GX" : 0,
            "SIM_APOS_GY" : 0,
            "SIM_APOS_GZ" : 0,
            "SIM_APOS_MODE" : 7, # CRUISE
            })

        for t in range(NTESTS):
            self.change_mode("FBWA")
            self.delay_sim_time(3, reason="mode change to stabilise")
            self.progress("Slow CRUISE Recovery test %u" % t)
            self.arm_vehicle(force=True)
            self.delay_sim_time(3, reason="recovery to begin")
            # reset target alt and heading using stick inputs
            self.set_rc(2, 1600)
            self.set_rc(2, 1500)
            self.set_rc(1, 1600)
            self.set_rc(1, 1500)
            self.wait_attitude(desroll=0, despitch=0, timeout=10, tolerance=10)
            self.wait_airspeed(target_airspeed-1, target_airspeed+1)
            final_alt = self.assert_receive_message('TERRAIN_REPORT').current_height
            if final_alt < 100:
                raise NotAchievedException(f"Final alt {final_alt:.1f}")
            self.disarm_vehicle(force=True)

        self.set_parameter("SIM_APOS_ENABLE", 0)
        self.arm_vehicle(force=True)
        self.change_mode("QLAND")
        self.wait_disarmed(timeout=300) # give quadplane a long time to land
