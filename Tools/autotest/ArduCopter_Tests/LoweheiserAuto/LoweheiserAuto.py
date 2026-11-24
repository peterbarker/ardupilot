'''
Copter autotest: LoweheiserAuto

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class LoweheiserAuto(TestGroup):
    @testcase(bucket="CopterTests2d")
    def LoweheiserAuto(self):
        '''Ensure the Loweheiser generator works as expected in auto-starter mode.'''

        gen_ctrl_ch = 9

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
            "LOG_DISARMED": 1
        })

        self.set_rc_from_map({
            gen_ctrl_ch: 1000,   # remember this is a switch position - stop
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

        self.context_collect('STATUSTEXT')

        # check prearms - should bounce due to generator not in correct state
        self.try_arm(result=False, expect_msg="requested state is not RUN")

        #######################################################################
        # Generator IDLE subtest.
        #######################################################################
        self.start_subtest("Setting generator to IDLE state.")
        self.set_rc(gen_ctrl_ch, 1500) # remember this is a switch position - idle
        self.wait_statustext("Generator MIDDLE", check_context=True)
        self.delay_sim_time(2, reason="generator to reach idle state")
        self.drain_mav()
        self.assert_received_message_field_values('EFI_STATUS', {
            "intake_manifold_pressure": 94,
            "exhaust_gas_temperature": float("nan"),
            "ignition_voltage": 12,
            "throttle_position": 40,
        }, epsilon=1.0)

        self.wait_message_field_values('EFI_STATUS', {
            "cylinder_head_temperature": 20,
        }, epsilon=5.0, timeout=10)

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
        self.wait_generator_speed_and_state(2000, 3000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE)

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
        self.wait_generator_speed_and_state(2000, 3000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE)

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
        self.set_rc(gen_ctrl_ch, 2000)
        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)

        self.set_safetyswitch_on()
        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)
        self.wait_message_field_values('EFI_STATUS', {
            # note that percent isn't honouring dead-zones...
            "throttle_position": 0,
            "rpm": 0,
        })

        self.set_safetyswitch_off()

        self.wait_message_field_values('EFI_STATUS', {
            # note that percent isn't honouring dead-zones...
            "throttle_position": 80,  # Throttle set by governor.
            "rpm": 8000,  # RPM for 50% throttle
        }, timeout=20)

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
        self.set_rc(gen_ctrl_ch, 2000)
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
