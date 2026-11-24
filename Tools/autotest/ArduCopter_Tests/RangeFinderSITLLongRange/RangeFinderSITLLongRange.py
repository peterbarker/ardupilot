'''
Copter autotest: RangeFinderSITLLongRange

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra
from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RangeFinderSITLLongRange(TestGroup):
    @testcase(bucket="CopterTests2c")
    def RangeFinderSITLLongRange(self):
        '''test rangefinder above 327m'''
        # FIXME: when we get a driver+simulator for a rangefinder with
        # >327m range change this to use that driver
        self.set_parameters({
            "RNGFND1_TYPE": 100,  # SITL
            "WP_SPD_UP": 10,    # m/s
            "RNGFND1_MAX": 7000,  # metres
        })
        self.reboot_sitl()

        text_good = "GOOD"
        text_high = "HIGH"

        rf_bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION

        alt = 3
        self.takeoff(alt, mode='GUIDED')
        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
        m = self.assert_receive_message('DISTANCE_SENSOR', verbose=True)
        got = m.current_distance * 0.01
        if abs(got - alt) > 1:
            raise NotAchievedException(f"Expected {alt}m range {got=}")

        self.send_statustext(text_good)

        alt = 635
        self.fly_guided_move_local(0, 0, alt)
        self.assert_sensor_state(rf_bit, present=True, enabled=True, healthy=True)
        m = self.assert_receive_message('DISTANCE_SENSOR', verbose=True)
        if abs(m.current_distance * 0.01 - alt) > 1:
            raise NotAchievedException(f"Expected {alt}m range")

        self.send_statustext(text_high)

        dfreader = self.dfreader_for_current_onboard_log()

        # fast descent!
        def hook(msg, m):
            if m.get_type() != 'HEARTBEAT':
                return
            # tell vehicle to only pay attention to the attitude:
            bitmask = 0
            bitmask |= mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
            bitmask |= mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
            bitmask |= mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
            bitmask |= mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE

            self.mav.mav.set_attitude_target_send(
                0, # time_boot_ms
                1, # target sysid
                1, # target compid
                0, # bitmask of things to ignore
                mavextra.euler_to_quat([
                    math.radians(-180),
                    math.radians(0),
                    math.radians(0)]), # att
                0, # roll rate  (rad/s)
                0, # pitch rate (rad/s)
                0, # yaw rate   (rad/s)
                1   # thrust, 0 to 1, translated to a climb/descent rate
            )

        self.install_message_hook_context(hook)

        self.wait_altitude(0, 30, timeout=200, relative=True)
        self.do_RTL()

        self.progress("Checking in/out of range markers in log")
        good = False
        while True:
            m = dfreader.recv_match(type=["MSG", "RFND"])
            if m is None:
                break
            m_type = m.get_type()
            if m_type == "RFND":
                if m.Dist < 600:
                    continue
                good = True
                break

        if not good:
            raise NotAchievedException("Did not see good alt")
