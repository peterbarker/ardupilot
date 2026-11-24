'''
Plane autotest: TestFlaps

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TestFlaps(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def TestFlaps(self):
        """Test flaps functionality."""
        filename = "flaps.txt"
        flaps_ch = 5
        flaps_ch_min = 1000
        flaps_ch_trim = 1500
        flaps_ch_max = 2000

        servo_ch = 5
        servo_ch_min = 1200
        servo_ch_trim = 1300
        servo_ch_max = 1800

        self.set_parameters({
            "SERVO%u_FUNCTION" % servo_ch: 3, # flapsauto
            "RC%u_OPTION" % flaps_ch: 208, # Flaps RCx_OPTION
            "LAND_FLAP_PERCNT": 50,
            "LOG_DISARMED": 1,
            "RTL_AUTOLAND": 1,

            "RC%u_MIN" % flaps_ch: flaps_ch_min,
            "RC%u_MAX" % flaps_ch: flaps_ch_max,
            "RC%u_TRIM" % flaps_ch: flaps_ch_trim,

            "SERVO%u_MIN" % servo_ch: servo_ch_min,
            "SERVO%u_MAX" % servo_ch: servo_ch_max,
            "SERVO%u_TRIM" % servo_ch: servo_ch_trim,
        })

        self.progress("check flaps are not deployed")
        self.set_rc(flaps_ch, flaps_ch_min)
        self.wait_servo_channel_value(servo_ch, servo_ch_min, timeout=3)
        self.progress("deploy the flaps")
        self.set_rc(flaps_ch, flaps_ch_max)
        tstart = self.get_sim_time()
        self.wait_servo_channel_value(servo_ch, servo_ch_max)
        tstop = self.get_sim_time_cached()
        delta_time = tstop - tstart
        delta_time_min = 0.5
        delta_time_max = 1.5
        if delta_time < delta_time_min or delta_time > delta_time_max:
            raise NotAchievedException((
                "Flaps Slew not working (%f seconds)" % (delta_time,)))
        self.progress("undeploy flaps")
        self.set_rc(flaps_ch, flaps_ch_min)
        self.wait_servo_channel_value(servo_ch, servo_ch_min)

        self.start_flying_mission(filename)
        self.start_subtest("flaps should deploy for landing")  # (RC input value used for position?!)
        self.wait_servo_channel_value(servo_ch, flaps_ch_trim, timeout=300)
        self.start_subtest("flaps should undeploy at the end")
        self.wait_servo_channel_value(servo_ch, servo_ch_min, timeout=30)

        self.progress("Flaps OK")

        # because we have used flaps, RC output 5 is now non-zero -
        # it's actually the SERVO5_MIN value of 1100 now.  The simulator
        # sees that as 100us above the zero-position (it is
        # 1000-to-2000 for flaps).  That slows the aircraft down!
        self.reboot_sitl()
