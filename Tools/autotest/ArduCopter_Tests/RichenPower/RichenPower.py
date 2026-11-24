'''
Copter autotest: RichenPower

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RichenPower(TestGroup):
    @testcase(bucket="CopterTests1e")
    def RichenPower(self):
        '''Test RichenPower generator'''
        self.set_parameters({
            "SERIAL5_PROTOCOL": 30,
            "SIM_RICH_ENABLE": 1,
            "SERVO8_FUNCTION": 42,
            "SIM_RICH_CTRL": 8,
            "RC9_OPTION": 85,
            "LOG_DISARMED": 1,
            "BATT2_MONITOR": 17,
            "GEN_TYPE": 3,
        })
        self.reboot_sitl()
        self.set_rc(9, 1000) # remember this is a switch position - stop
        self.customise_SITL_commandline(["--serial5=sim:richenpower"])
        self.wait_statustext("requested state is not RUN", timeout=60)

        self.set_message_rate_hz("GENERATOR_STATUS", 10)

        self.wait_generator_speed_and_state(0, 0, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_OFF)

        self.context_collect('STATUSTEXT')
        self.set_rc(9, 2000) # remember this is a switch position - run
        self.wait_statustext("Generator HIGH", check_context=True)
        self.set_rc(9, 1000) # remember this is a switch position - stop
        self.wait_statustext("requested state is not RUN", timeout=200)

        self.set_rc(9, 1500) # remember this is a switch position - idle
        self.wait_generator_speed_and_state(3000, 8000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE)

        self.set_rc(9, 2000) # remember this is a switch position - run
#        self.wait_generator_speed_and_state(3000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_WARMING_UP)

        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)

        bs = self.assert_receive_message(
            "BATTERY_STATUS",
            condition="BATTERY_STATUS.id==1",  # id is zero-indexed
        )
        self.progress("Received battery status: %s" % str(bs))
        want_bs_volt = 50000
        if bs.voltages[0] != want_bs_volt:
            raise NotAchievedException("Battery voltage not as expected (want=%f) got=(%f)" % (want_bs_volt, bs.voltages[0],))

        self.progress("Moving *back* to idle")
        self.set_rc(9, 1500) # remember this is a switch position - idle
        self.wait_generator_speed_and_state(3000, 10000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_IDLE)

        self.progress("Moving *back* to run")
        self.set_rc(9, 2000) # remember this is a switch position - run
        self.wait_generator_speed_and_state(8000, 30000, mavutil.mavlink.MAV_GENERATOR_STATUS_FLAG_GENERATING)

        self.set_message_rate_hz("GENERATOR_STATUS", -1)
        self.set_parameter("LOG_DISARMED", 0)
        if not self.current_onboard_log_contains_message("RICH"):
            raise NotAchievedException("Did not find expected RICH message")
