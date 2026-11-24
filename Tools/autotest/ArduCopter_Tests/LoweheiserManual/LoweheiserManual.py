'''
Copter autotest: LoweheiserManual

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class LoweheiserManual(TestGroup):
    @testcase(bucket="CopterTests2c")
    def LoweheiserManual(self):
        '''Ensure the Loweheiser generator works as expected in manual starter mode.'''

        gen_ctrl_ch = 9
        loweheiser_man_throt_ch = 10
        loweheiser_man_start_ch = 11

        self.set_parameters({
            "SERIAL5_PROTOCOL": 2,    # mavlink
            "MAV3_OPTIONS": 2,        # private
            "GEN_TYPE": 4,            # loweheiser
            "EFI_TYPE": 4,            # loweheiser
            "SIM_EFI_TYPE": 2,        # loweheiser sim
            "BATT2_MONITOR": 17,  # generator (elec)
            "BATT3_MONITOR": 18,  # generator (fuel-level)
            "BATT3_CAPACITY": 10000,  # generator (fuel) in mL
            f"RC{gen_ctrl_ch}_OPTION": 85,  # generator control
            "LOG_DISARMED": 1,
            "RC%u_OPTION" % loweheiser_man_throt_ch: 218,  # loweheiser manual throttle control
            "RC%u_DZ" % loweheiser_man_throt_ch: 20,
            "RC%u_TRIM" % loweheiser_man_throt_ch: 1000,
            "RC%u_MIN" % loweheiser_man_throt_ch: 1000,
            "RC%u_MAX" % loweheiser_man_throt_ch: 2000,
            "RC%u_OPTION" % loweheiser_man_start_ch: 111, # loweheiser starter channel
        })

        self.set_rc_from_map({
            gen_ctrl_ch: 1000,   # remember this is a switch position - stop
            loweheiser_man_throt_ch: 1000,  # zero throttle
            loweheiser_man_start_ch: 1000,  # the starter is off
        })

        self.customise_SITL_commandline(["--serial5=sim:loweheiser"])

        self.set_parameters({
            "GEN_L_IDLE_TH": 25,
            "GEN_L_IDLE_TH_H": 40,
            "GEN_L_RUN_TEMP": 60,
            "GEN_L_IDLE_TEMP": 80,
        })

        self.reboot_sitl()

        self.assert_parameter_value("GEN_L_IDLE_TH", 25)

        self.delay_sim_time(10, reason="generator driver to initialise")  # so we can actually receive messages...

        #######################################################################
        # Generator OFF subtest.
        #######################################################################
        self.start_subtest("Checking GENERATOR_STATUS while OFF.")
        self.set_message_rate_hz("GENERATOR_STATUS", 10)

        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)

        self.start_subtest("Checking EFI_STATUS while OFF.")
        self.set_message_rate_hz("EFI_STATUS", 10)
        self.delay_sim_time(1, reason="message rate to take effect")
        self.assert_received_message_field_values('EFI_STATUS', {
            "health": 1,
            "ecu_index": 1.0,
            "rpm": 0.0,
            "fuel_consumed": 0,
            "fuel_flow": float("nan"),
            "engine_load": 0.0,
            "throttle_position": 0.0,
            "spark_dwell_time": 0.0,
            "barometric_pressure": 0.0,
            "intake_manifold_pressure": float("nan"),
            "intake_manifold_temperature": float("nan"),
            "cylinder_head_temperature": float("nan"),
            "ignition_timing": 0.0,
            "injection_time": float("nan"),
            "exhaust_gas_temperature": float("nan"),
            "throttle_out": 0.0,
            "pt_compensation": 0.0,
            "ignition_voltage": 0,  # As per the spec, 0 means "Unknown".
        }, epsilon=1.0)
        self.assert_received_message_field_values('GENERATOR_STATUS', {
            "status": 1,
            "generator_speed": 0,
            "battery_current": -0.30000001192092896,
            "load_current": 10.119999885559082,
            "power_generated": 521.0,
            "bus_voltage": 50,
            "rectifier_temperature": 32767,
            "bat_current_setpoint": float("nan"),
            "generator_temperature": 32767,
            "runtime": 0,
            "time_until_maintenance": 300*60*60,
        })

        rc_dz = self.get_parameter('RC%u_DZ' % loweheiser_man_throt_ch)
        rc_trim = int(self.get_parameter('RC%u_TRIM' % loweheiser_man_throt_ch))
        rc_min = self.get_parameter('RC%u_MIN' % loweheiser_man_throt_ch)
        rc_max = int(self.get_parameter('RC%u_MAX' % loweheiser_man_throt_ch))

        self.progress("Ensuring 1000 and rc_trim all leave throttle out 0")
        for i in 1000, rc_trim:
            self.progress("Checking %u pwm" % i)
            self.set_rc(loweheiser_man_throt_ch, i)
            self.delay_sim_time(1, reason="throttle change to propagate")
            self.assert_received_message_field_values('EFI_STATUS', {
                "throttle_position": 0,
                "rpm": 0,
            })

        self.context_collect('STATUSTEXT')
        # check prearms - should bounce due to generator not in correct state
        self.try_arm(result=False, expect_msg="requested state is not RUN")

        #######################################################################
        # Generator IDLE subtest.
        #######################################################################
        self.start_subtest("Setting generator to IDLE state.")
        self.set_rc(gen_ctrl_ch, 1500) # remember this is a switch position - idle
        self.wait_statustext("Generator MIDDLE", check_context=True)
        self.delay_sim_time(5, reason="generator to reach idle state")

        # Ensure the generator has not auto-started.
        self.assert_received_message_field_values('EFI_STATUS', {
            "throttle_out": 0,
            "rpm": 0,
        })

        # at 50 percent throttle should not start until user triggers starter
        pwm_for_fifty_percent_throttle = int(rc_min + rc_dz + int((rc_max-rc_min-rc_dz)/2))
        self.progress("Using PWM of %u for 50 percent throttle" % pwm_for_fifty_percent_throttle)
        self.set_rc(loweheiser_man_throt_ch, pwm_for_fifty_percent_throttle)
        self.wait_message_field_values('EFI_STATUS', {
            # note that percent isn't honouring dead-zones...
            "throttle_position": 51,  # magic fixed throttle value from AP_Generator_Loweheiser.cpp
            "rpm": 0,
        }, timeout=20)

        self.set_rc(loweheiser_man_start_ch, 2000)  # Run the starter for a few seconds.

        self.drain_mav()
        self.assert_received_message_field_values('EFI_STATUS', {
            "intake_manifold_pressure": 94,
            "exhaust_gas_temperature": float("nan"),
            "ignition_voltage": 12,
        }, epsilon=1.0)

        self.wait_message_field_values('EFI_STATUS', {
            "cylinder_head_temperature": 20,
        }, epsilon=5.0, timeout=10)

        self.set_rc(loweheiser_man_start_ch, 1000)

        self.assert_received_message_field_values('GENERATOR_STATUS', {
            # Commenting out "status" argument.
            # For some reason the test suite can't accept an integer value. Says it's not part of the FLAG.
            # "status": mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE,
            "battery_current": -0.30000001192092896,
            "load_current": 10.119999885559082,
            "power_generated": 521.0,
            "bus_voltage": 50,
            "rectifier_temperature": 32767,
            "bat_current_setpoint": float("nan"),
            "generator_temperature": 32767,
            "runtime": 2,
            "time_until_maintenance": 300*60*60 - 2,
        })

        self.progress("Generator at idle should not run governor and use throttle input")
        self.wait_message_field_values('EFI_STATUS', {
            # note that percent isn't honouring dead-zones...
            "throttle_position": 51,  # magic fixed throttle value from AP_Generator_Loweheiser.cpp
            "rpm": 4080,  # RPM for 50% throttle
        }, timeout=20)

        #######################################################################
        # Generator RUN subtest.
        #######################################################################
        self.start_subtest("Setting generator to RUN state.")
        self.set_rc(gen_ctrl_ch, 2000) # remember this is a switch position - run
        self.wait_statustext("Generator HIGH", check_context=True)

        # check prearms - should bounce due to generator too cold
        self.try_arm(result=False, expect_msg="Generator warming up")

        self.set_rc(gen_ctrl_ch, 1000) # remember this is a switch position - stop
        self.wait_statustext("requested state is not RUN", timeout=200)

        self.set_rc(gen_ctrl_ch, 1500) # remember this is a switch position - idle

        # ask for a throttle whose steady-state RPM is inside the band we
        # check for below.  At 50% throttle the engine settles at 4080rpm,
        # so the only time it is in-band is while it is spinning up - a
        # race we lose when the engine hasn't wound down far enough while
        # stopped:
        pwm_for_thirty_percent_throttle = int(rc_min + int((rc_max-rc_min)*0.3))
        self.progress("Using PWM of %u for 30 percent throttle" % pwm_for_thirty_percent_throttle)
        self.set_rc(loweheiser_man_throt_ch, pwm_for_thirty_percent_throttle)

        self.set_rc(loweheiser_man_start_ch, 2000)
        self.wait_generator_speed_and_state(2000, 3000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE)
        self.set_rc(loweheiser_man_start_ch, 1000)

        self.set_rc(loweheiser_man_throt_ch, pwm_for_fifty_percent_throttle)

        self.set_rc(gen_ctrl_ch, 2000) # remember this is a switch position - run

        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)

        bs = self.assert_receive_message(
            "BATTERY_STATUS",
            condition="BATTERY_STATUS.id==1",  # id is zero-indexed
            timeout=1,
            very_verbose=True,
        )
        if bs is None:
            raise NotAchievedException("Did not receive BATTERY_STATUS")
        self.progress("Received battery status: %s" % str(bs))
        want_bs_volt = 50000
        if bs.voltages[0] != want_bs_volt:
            raise NotAchievedException("Battery voltage not as expected (want=%f) got=(%f)" % (want_bs_volt, bs.voltages[0],))

        self.progress("Checking battery remaining")
        bs = self.assert_receive_message(
            "BATTERY_STATUS",
            condition="BATTERY_STATUS.id==2",  # id is zero-indexed
            timeout=1,
            very_verbose=True,
        )
        self.progress("Waiting for some fuel to be consumed...")
        self.wait_message_field_values("BATTERY_STATUS", {
            "id": 2,
            "battery_remaining": bs.battery_remaining-1,
        }, timeout=100)

        bs2 = self.assert_receive_message(
            "BATTERY_STATUS",
            condition="BATTERY_STATUS.id==2",  # id is zero-indexed
            timeout=1,
            very_verbose=True,
        )
        if bs2.battery_remaining >= bs.battery_remaining:
            raise NotAchievedException("Expected battery remaining to drop")
        if bs2.current_consumed <= bs.current_consumed:
            raise NotAchievedException("Expected energy consumed to rise")

        self.progress("Checking battery reset")
        batt_reset_values = [(25, 24),
                             (50, 49),
                             (63, 62),
                             (87, 86),
                             (100, 99)]

        for (reset_val, return_val) in batt_reset_values:
            self.run_cmd(mavutil.mavlink.MAV_CMD_BATTERY_RESET,
                         (1 << 2), # param1 - bitmask of batteries to reset
                         reset_val, # level to reset to
                         0, # param3
                         0, # param4
                         0, # param5
                         0, # param6
                         0 # param7
                         )
            self.wait_message_field_values("BATTERY_STATUS", {
                "id": 2,
                "battery_remaining": return_val,
            }, timeout=5)

        self.progress("Moving *back* to idle")
        self.set_rc(gen_ctrl_ch, 1500) # remember this is a switch position - idle
        self.wait_generator_speed_and_state(3000, 10000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE)

        self.progress("Moving *back* to run")
        self.set_rc(gen_ctrl_ch, 2000) # remember this is a switch position - run
        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)

        self.set_parameter("LOG_DISARMED", 0)
        if not self.current_onboard_log_contains_message("LOEG"):
            raise NotAchievedException("Did not find expected LOEG message in .bin log")
        if not self.current_onboard_log_contains_message("LOEC"):
            raise NotAchievedException("Did not find expected LOEC message in .bin log")

        self.set_parameter("LOG_DISARMED", 1)  # Re-start logging, to help with debugging.
        self.progress("Stopping generator")
        self.set_rc(gen_ctrl_ch, 1000)

        self.wait_statustext("Cooling down")

        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)

        #######################################################################
        # E-stop subtest.
        #######################################################################
        self.start_subtest("Checking safety switch estop")
        # Start the engine.
        self.set_rc(gen_ctrl_ch, 1500)
        self.set_rc(loweheiser_man_start_ch, 2000)
        self.wait_message_field_values('EFI_STATUS', {
            # note that percent isn't honouring dead-zones...
            "rpm": 4000,  # RPM for 50% throttle
        }, timeout=20)
        self.set_rc(loweheiser_man_start_ch, 1000)

        self.set_safetyswitch_on()
        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)
        self.wait_message_field_values('EFI_STATUS', {
            # note that percent isn't honouring dead-zones...
            "throttle_position": 0,
            "rpm": 0,
        })

        self.set_safetyswitch_off()

        self.set_rc(gen_ctrl_ch, 1500)
        self.set_rc(loweheiser_man_start_ch, 2000)
        self.wait_message_field_values('EFI_STATUS', {
            # note that percent isn't honouring dead-zones...
            "throttle_position": 51,  # magic fixed throttle value from AP_Generator_Loweheiser.cpp
            "rpm": 4000,  # RPM for 50% throttle
        }, timeout=20)
        self.set_rc(loweheiser_man_start_ch, 1000)

        # stop generator:
        self.set_rc(gen_ctrl_ch, 1000)
        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)

        #######################################################################
        # Battery failsafe subtest.
        #######################################################################
        self.start_subtest("Battery Failsafes")
        self.context_push()
        batt3_capacity = 500
        batt3_low_mah = 100
        batt3_low_consumed_mah = batt3_capacity - batt3_low_mah
        self.set_parameters({
            "BATT3_CAPACITY": batt3_capacity,
            "BATT3_LOW_MAH": batt3_low_mah,
            "BATT3_CRT_MAH": 50,
            "BATT3_FS_LOW_ACT": 2, # RTL
            "BATT3_FS_CRT_ACT": 1, # LAND
            "BATT3_LOW_VOLT": 0,
        })
        self.reboot_sitl()

        # Start the generator anew.
        self.set_rc(gen_ctrl_ch, 1500)
        self.set_rc(loweheiser_man_start_ch, 2000)
        self.delay_sim_time(5, reason="generator starter to engage")
        self.set_rc(loweheiser_man_start_ch, 1000)
        self.set_rc(gen_ctrl_ch, 2000)
        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)
        self.takeoff(10, mode='GUIDED')

        # the generator has been running since before takeoff; make
        # sure fuel is being consumed.  This is a wait rather than an
        # instantaneous check as exactly how much has been consumed by
        # now depends on incidental harness timing:
        self.wait_efi_fuel_consumed(100)

        self.fly_guided_move_local(100, 100, 20)

        self.wait_mode('RTL', timeout=300)

        second_efi_status = self.assert_receive_message('EFI_STATUS', verbose=True)

        if second_efi_status.fuel_consumed < batt3_low_consumed_mah:
            raise NotAchievedException("Unexpected fuel consumed value after failsafe (%f)" % second_efi_status.fuel_consumed)

        self.wait_mode('LAND', timeout=300)
        self.wait_disarmed()

        self.context_pop()
        self.reboot_sitl()

        self.set_message_rate_hz("EFI_STATUS", -1)
        self.set_message_rate_hz("GENERATOR_STATUS", -1)
