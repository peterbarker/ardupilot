'''
Tracker autotest: GUIDED

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GUIDED(TestGroup):
    def achieve_attitude(self, desyaw, despitch, tolerance=1, target_system=2, target_component=1):
        '''use set_attitude_target to achieve desyaw / despitch'''
        tstart = self.get_sim_time()
        last_attitude_target_sent = 0
        last_debug = 0
        self.progress("Using set_attitude_target to achieve attitude")
        while True:
            now = self.get_sim_time()
            if now - tstart > 60:
                raise NotAchievedException("Did not achieve attitude")
            if now - last_attitude_target_sent > 0.5:
                last_attitude_target_sent = now
                type_mask = (
                    1 << 0 | # ignore roll rate
                    1 << 6 # ignore throttle
                )
                self.mav.mav.set_attitude_target_send(
                    0, # time_boot_ms
                    target_system, # target sysid
                    target_component, # target compid
                    type_mask, # bitmask of things to ignore
                    mavextra.euler_to_quat([0,
                                            math.radians(despitch),
                                            math.radians(desyaw)]), # att
                    0, # yaw rate (rad/s)
                    0, # pitch rate
                    0, # yaw rate
                    0) # thrust, 0 to 1, translated to a climb/descent rate
            m = self.assert_receive_message('ATTITUDE', timeout=2)
            if now - last_debug > 1:
                last_debug = now
                self.progress("yaw=%f desyaw=%f pitch=%f despitch=%f" %
                              (math.degrees(m.yaw), desyaw,
                               math.degrees(m.pitch), despitch))
            yaw_ok = abs(math.degrees(m.yaw) - desyaw) < tolerance
            pitch_ok = abs(math.degrees(m.pitch) - despitch) < tolerance
            if yaw_ok and pitch_ok:
                self.progress("Achieved attitude")
                break

    @testcase(bucket="Tracker")
    def GUIDED(self):
        '''Test GUIDED mode'''
        self.reboot_sitl() # temporary hack around control issues
        self.change_mode(4) # "GUIDED"
        self.achieve_attitude(desyaw=10, despitch=30)
        self.achieve_attitude(desyaw=0, despitch=0)
        self.achieve_attitude(desyaw=45, despitch=10)

    @testcase(bucket="Tracker")
    def GUIDEDHoldsAttitudeOnEntry(self):
        '''ensure entering GUIDED holds the current attitude until a target arrives'''
        self.reboot_sitl()
        self.set_parameters({
            # keep pitch away from its limits while scanning
            "SCAN_SPEED_PIT": 0,
            # build up integrator for GUIDED to discard
            "YAW2SRV_I": 0.02,
            "PITCH2SRV_I": 0.02,
        })

        def enter_guided_from_auto_and_check_hold(stale_target_yaw):
            # scan away from the stale GUIDED target
            self.change_mode(2)  # "SCAN"
            tstart = self.get_sim_time()
            while True:
                # SCAN may have to go the long way round
                if self.get_sim_time_cached() - tstart > 240:
                    raise NotAchievedException("Did not scan away from yaw=%f" % stale_target_yaw)
                m = self.assert_receive_message('ATTITUDE')
                if abs(mavextra.angle_diff(math.degrees(m.yaw), stale_target_yaw)) > 60:
                    break

            # with no vehicle position AUTO leaves the servos where they are
            self.change_mode("AUTO")
            self.wait_message_field_values('ATTITUDE', {
                'yawspeed': 0,
                'pitchspeed': 0,
            }, epsilon=math.radians(0.5), minimum_duration=2, timeout=60)

            self.change_mode(4)  # "GUIDED"
            m = self.assert_receive_message('ATTITUDE')
            entry_yaw = math.degrees(m.yaw)
            entry_pitch = math.degrees(m.pitch)
            self.progress("Entered GUIDED at yaw=%f pitch=%f" % (entry_yaw, entry_pitch))
            tstart = self.get_sim_time()
            while self.get_sim_time_cached() - tstart < 10:
                m = self.assert_receive_message('ATTITUDE')
                yaw_error = abs(mavextra.angle_diff(math.degrees(m.yaw), entry_yaw))
                pitch_error = abs(math.degrees(m.pitch) - entry_pitch)
                if yaw_error > 2 or pitch_error > 2:
                    raise NotAchievedException(
                        "Moved after entering GUIDED: yaw=%f pitch=%f" %
                        (math.degrees(m.yaw), math.degrees(m.pitch)))

        # first entry after boot must not slew to the default (north, level) target
        enter_guided_from_auto_and_check_hold(0)

        # re-entry must not slew to the previous GUIDED target; the I
        # gains can make the tracker oscillate around this target
        self.set_parameters({"YAW2SRV_I": 0, "PITCH2SRV_I": 0})
        self.achieve_attitude(desyaw=-90, despitch=10)
        self.set_parameters({"YAW2SRV_I": 0.02, "PITCH2SRV_I": 0.02})
        enter_guided_from_auto_and_check_hold(-90)
