'''
Copter autotest: MountMAVLinkTargetRefresh

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil
from pymavlink import quaternion

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MountMAVLinkTargetRefresh(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MountMAVLinkTargetRefresh(self):
        '''send changed gimbal commands immediately and refresh retained commands'''
        self.set_parameters({"MNT1_TYPE": 6, "MNT1_TARG_RATE": 1})
        self.reboot_sitl()
        self.context_collect('GIMBAL_DEVICE_SET_ATTITUDE')
        tstart = self.get_sim_time()
        while not self.context_collection('GIMBAL_DEVICE_SET_ATTITUDE'):
            if self.get_sim_time_cached() - tstart > 30:
                raise NotAchievedException("MAVLink gimbal was not discovered")
            old_sysid, old_compid = self.mav.mav.srcSystem, self.mav.mav.srcComponent
            try:
                self.mav.mav.srcSystem = self.sysid_thismav()
                self.mav.mav.srcComponent = mavutil.mavlink.MAV_COMP_ID_GIMBAL
                self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GIMBAL, 0, 0, 0, 0)
                self.mav.mav.gimbal_device_information_send(
                    0, b"ArduPilot", b"Test", b"", 1, 1, 1, 0, 0,
                    -math.pi, math.pi, -math.pi / 2, math.pi / 2, -math.pi, math.pi,
                    cap_flags2=mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL)
            finally:
                self.mav.mav.srcSystem, self.mav.mav.srcComponent = old_sysid, old_compid
            self.delay_sim_time(1, reason="discover test gimbal")

        if self.get_parameter("MNT1_ATT_RATE") != 50:
            raise NotAchievedException("Unexpected MNT1_ATT_RATE default")
        attitude_rate = self.measure_message_rate('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', timeout=5)
        if abs(attitude_rate - 50) > 5:
            raise NotAchievedException("Default gimbal attitude rate: want=50Hz got=%fHz" % attitude_rate)

        def command(pitch, pitch_rate=float('nan'), flags=0):
            self.context_clear_collection('GIMBAL_DEVICE_SET_ATTITUDE')
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                         p1=pitch, p2=0, p3=pitch_rate, p4=0, p5=flags)
            start = self.get_sim_time_cached()
            while True:
                if self.get_sim_time_cached() - start > 0.4:
                    raise NotAchievedException("Changed gimbal target waited for the refresh interval")
                # An old periodic refresh may already be in flight when the
                # command is sent; only the requested target proves delivery.
                for msg in self.context_collection('GIMBAL_DEVICE_SET_ATTITUDE'):
                    yaw_lock = bool(msg.flags & mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_LOCK)
                    if yaw_lock != bool(flags & mavutil.mavlink.GIMBAL_MANAGER_FLAGS_YAW_LOCK):
                        continue
                    if not math.isnan(pitch):
                        if math.isnan(msg.q[0]):
                            continue
                        got_pitch = math.degrees(quaternion.Quaternion(msg.q).euler[1])
                        if abs(got_pitch - pitch) <= 0.1:
                            return msg
                    elif abs(math.degrees(msg.angular_velocity_y) - pitch_rate) <= 0.1:
                        return msg
                self.mav.recv_match(blocking=True, timeout=0.01)

        for pitch in (-10, -20, -30):
            msg = command(pitch)
            got_pitch = math.degrees(quaternion.Quaternion(msg.q).euler[1])
            if abs(got_pitch - pitch) > 0.1:
                raise NotAchievedException("Wrong gimbal angle target")
        msg = command(-30, flags=mavutil.mavlink.GIMBAL_MANAGER_FLAGS_YAW_LOCK)
        if not msg.flags & mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_LOCK:
            raise NotAchievedException("Yaw frame change was not sent")
        for rate in (5, 0):
            msg = command(float('nan'), pitch_rate=rate)
            if abs(math.degrees(msg.angular_velocity_y) - rate) > 0.1:
                raise NotAchievedException("Wrong gimbal rate target")

        self.context_clear_collection('GIMBAL_DEVICE_SET_ATTITUDE')
        self.delay_sim_time(2.2, reason="measure retained target refresh")
        count = len(self.context_collection('GIMBAL_DEVICE_SET_ATTITUDE'))
        if count != 2:
            raise NotAchievedException("Unchanged 1Hz target refreshed %u times in 2.2s" % count)
        self.set_parameter("MNT1_TARG_RATE", 0)
        self.context_clear_collection('GIMBAL_DEVICE_SET_ATTITUDE')
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                     p1=-40, p2=0, p3=float('nan'), p4=float('nan'))
        self.delay_sim_time(1.2, reason="check disabled target transmission")
        if self.context_collection('GIMBAL_DEVICE_SET_ATTITUDE'):
            raise NotAchievedException("Disabled gimbal target was sent")
        self.context_clear_collection('GIMBAL_DEVICE_SET_ATTITUDE')
        self.set_parameter("MNT1_TARG_RATE", 1)
        start = self.get_sim_time_cached()
        while not self.context_collection('GIMBAL_DEVICE_SET_ATTITUDE'):
            if self.get_sim_time_cached() - start > 1.5:
                raise NotAchievedException("Re-enabled gimbal target was not sent")
            self.mav.recv_match(blocking=True, timeout=0.01)

        self.context_collect('COMMAND_INT')
        here = self.get_location(frame=AltFrame.ABSOLUTE)
        for offset in (1000, 2000):
            self.context_clear_collection('COMMAND_INT')
            self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                             x=int(here.lat * 1e7) + offset, y=int(here.lng * 1e7), z=600,
                             frame=mavutil.mavlink.MAV_FRAME_GLOBAL)
            start = self.get_sim_time_cached()
            while True:
                if self.get_sim_time_cached() - start > 0.4:
                    raise NotAchievedException("Changed ROI waited for the refresh interval")
                if any(msg.command == mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION and
                       msg.x == int(here.lat * 1e7) + offset
                       for msg in self.context_collection('COMMAND_INT')):
                    break
                self.mav.recv_match(blocking=True, timeout=0.01)
        self.context_clear_collection('GIMBAL_DEVICE_SET_ATTITUDE')
        self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_RETRACT)
        self.delay_sim_time(0.2, reason="check retract is sent immediately")
        messages = self.context_collection('GIMBAL_DEVICE_SET_ATTITUDE')
        if not messages or not messages[-1].flags & mavutil.mavlink.GIMBAL_DEVICE_FLAGS_RETRACT:
            raise NotAchievedException("Retract mode was not sent immediately")
