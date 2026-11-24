'''
Copter autotest: RangeFinderDriversLongRange

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RangeFinderDriversLongRange(TestGroup):
    @testcase(bucket="CopterTests2c")
    def RangeFinderDriversLongRange(self):
        '''test rangefinder above 327m'''
        # FIXME: when we get a driver+simulator for a rangefinder with
        # >327m range change this to use that driver
        self.set_parameters({
            "SERIAL4_PROTOCOL": 9,
            "RNGFND1_TYPE": 19,  # BenewakeTF02
            "WP_SPD_UP": 10,  # m/s
        })
        self.customise_SITL_commandline([
            "--serial4=sim:benewake_tf02",
        ])

        text_good = "GOOD"
        text_out_of_range_high = "OUT_OF_RANGE_HIGH"

        rf_bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION

        alt = 3
        self.takeoff(alt, mode='GUIDED')
        self.assert_parameter_value("RNGFND1_MAX", 7.00)
        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
        m = self.assert_receive_message('DISTANCE_SENSOR', verbose=True)
        if abs(m.current_distance * 0.01 - alt) > 1:
            raise NotAchievedException(f"Expected {alt}m range got range={m.current_distance * 0.01}")
        self.assert_parameter_value("RNGFND1_MAX", m.max_distance * 0.01, epsilon=0.00001)
        self.assert_parameter_value("RNGFND1_MIN", m.min_distance * 0.01, epsilon=0.00001)

        self.send_statustext(text_good)

        alt = 10
        self.fly_guided_move_local(0, 0, alt)
        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
        m = self.assert_receive_message('DISTANCE_SENSOR', verbose=True)
        if abs(m.current_distance * 0.01 - alt) > 1:
            raise NotAchievedException(f"Expected {alt}m range")
        self.assert_parameter_value("RNGFND1_MAX", m.max_distance * 0.01, epsilon=0.00001)
        self.assert_parameter_value("RNGFND1_MIN", m.min_distance * 0.01, epsilon=0.00001)
        self.send_statustext(text_out_of_range_high)

        self.progress("Moving the goalposts")
        self.set_parameter("RNGFND1_MAX", 20)
        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
        m = self.assert_receive_message('DISTANCE_SENSOR', verbose=True)
        if abs(m.current_distance * 0.01 - alt) > 1:
            raise NotAchievedException(f"Expected {alt}m range")
        self.assert_parameter_value("RNGFND1_MAX", m.max_distance * 0.01, epsilon=0.00001)
        self.assert_parameter_value("RNGFND1_MIN", m.min_distance * 0.01, epsilon=0.00001)
        self.send_statustext(text_good)
        self.delay_sim_time(2, reason="rangefinder log marker to be written")

        dfreader = self.dfreader_for_current_onboard_log()

        self.do_RTL()

        self.progress("Checking in/out of range markers in log")
        required_range = None
        count = 0
        while True:
            m = dfreader.recv_match(type=["MSG", "RFND"])
            if m is None:
                break
            m_type = m.get_type()
            if m_type == "MSG":
                for t in [text_good, text_out_of_range_high]:
                    if t in m.Message:
                        required_range = t
                continue
            if m_type == "RFND":
                if required_range is None:
                    continue
                if required_range == text_good:
                    required_state = 4
                elif required_range == text_out_of_range_high:
                    required_state = 3
                else:
                    raise ValueError(f"Unexpected text {required_range}")
                if m.Stat != required_state:
                    raise NotAchievedException(f"Not in expected state want={required_state} got={m.Stat} dist={m.Dist}")
                self.progress(f"In expected state {required_range}")
                required_range = None
                count += 1

        if count < 3:
            raise NotAchievedException("Did not see range markers")
