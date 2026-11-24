'''
Rover autotest: Sprayer

AP_FLAKE8_CLEAN
'''

import operator

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class Sprayer(TestGroup):
    @testcase(bucket="Rover")
    def Sprayer(self):
        """Test sprayer functionality."""
        rc_ch = 5
        pump_ch = 5
        spinner_ch = 6
        pump_ch_min = 1050
        pump_ch_trim = 1520
        pump_ch_max = 1950
        spinner_ch_min = 975
        spinner_ch_trim = 1510
        spinner_ch_max = 1975

        self.set_parameters({
            "SPRAY_ENABLE": 1,

            "SERVO%u_FUNCTION" % pump_ch: 22,
            "SERVO%u_MIN" % pump_ch: pump_ch_min,
            "SERVO%u_TRIM" % pump_ch: pump_ch_trim,
            "SERVO%u_MAX" % pump_ch: pump_ch_max,

            "SERVO%u_FUNCTION" % spinner_ch: 23,
            "SERVO%u_MIN" % spinner_ch: spinner_ch_min,
            "SERVO%u_TRIM" % spinner_ch: spinner_ch_trim,
            "SERVO%u_MAX" % spinner_ch: spinner_ch_max,

            "SIM_SPR_ENABLE": 1,
            "SIM_SPR_PUMP": pump_ch,
            "SIM_SPR_SPIN": spinner_ch,

            "RC%u_OPTION" % rc_ch: 15,
            "LOG_DISARMED": 1,
        })

        self.reboot_sitl()

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.progress("test boot-up state - it's zero-output!")
        self.wait_servo_channel_value(spinner_ch, 0)
        self.wait_servo_channel_value(pump_ch, 0)

        self.progress("Enable sprayer")
        self.set_rc(rc_ch, 2000)

        self.progress("Testing zero-speed state")
        self.wait_servo_channel_value(spinner_ch, spinner_ch_min)
        self.wait_servo_channel_value(pump_ch, pump_ch_min)

        self.progress("Testing turning it off")
        self.set_rc(rc_ch, 1000)
        self.wait_servo_channel_value(spinner_ch, spinner_ch_min)
        self.wait_servo_channel_value(pump_ch, pump_ch_min)

        self.progress("Testing turning it back on")
        self.set_rc(rc_ch, 2000)
        self.wait_servo_channel_value(spinner_ch, spinner_ch_min)
        self.wait_servo_channel_value(pump_ch, pump_ch_min)

        self.progress("Testing speed-ramping")
        self.set_rc(3, 1700) # start driving forward

        # this is somewhat empirical...
        self.wait_servo_channel_value(pump_ch, 1695, timeout=60)

        self.progress("Turning it off again")
        self.set_rc(rc_ch, 1000)
        self.wait_servo_channel_value(spinner_ch, spinner_ch_min)
        self.wait_servo_channel_value(pump_ch, pump_ch_min)

        self.start_subtest("Sprayer Mission")
        self.load_mission("sprayer-mission.txt")
        self.change_mode("AUTO")
#            self.send_debug_trap()
        self.progress("Waiting for sprayer to start")
        self.wait_servo_channel_value(pump_ch, 1250, timeout=60, comparator=operator.gt)
        self.progress("Waiting for sprayer to stop")
        self.wait_servo_channel_value(pump_ch, pump_ch_min, timeout=120)

        self.start_subtest("Checking mavlink commands")
        self.change_mode("MANUAL")
        self.progress("Starting Sprayer")
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SPRAYER, p1=1)

        self.progress("Testing speed-ramping")
        self.set_rc(3, 1700) # start driving forward
        self.wait_servo_channel_value(pump_ch, 1690, timeout=60, comparator=operator.gt)
        self.start_subtest("Stopping Sprayer")
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SPRAYER, p1=0)
        self.wait_servo_channel_value(pump_ch, pump_ch_min)
        self.set_rc(3, 1000) # stop driving forward

        self.progress("Sprayer OK")
        self.disarm_vehicle()
