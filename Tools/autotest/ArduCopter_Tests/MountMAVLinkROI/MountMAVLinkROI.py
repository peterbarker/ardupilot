'''
Copter autotest: MountMAVLinkROI

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil
from pymavlink import quaternion

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MountMAVLinkROI(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MountMAVLinkROI(self):
        '''route ROI and angle commands independently through two MAVLink mounts'''
        self.set_parameters({"MNT1_TYPE": 6, "MNT2_TYPE": 6,
                             "MNT1_TARG_RATE": 5, "MNT2_TARG_RATE": 5})
        self.reboot_sitl()
        components = (mavutil.mavlink.MAV_COMP_ID_GIMBAL, mavutil.mavlink.MAV_COMP_ID_GIMBAL2)
        self.context_collect('GIMBAL_DEVICE_SET_ATTITUDE')
        self.context_collect('COMMAND_INT')
        tstart = self.get_sim_time()
        while not all(any(msg.target_component == component
                          for msg in self.context_collection('GIMBAL_DEVICE_SET_ATTITUDE'))
                      for component in components):
            if self.get_sim_time_cached() - tstart > 30:
                raise NotAchievedException("MAVLink gimbals were not discovered")
            old_sysid, old_compid = self.mav.mav.srcSystem, self.mav.mav.srcComponent
            try:
                self.mav.mav.srcSystem = self.sysid_thismav()
                for component in components:
                    self.mav.mav.srcComponent = component
                    self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GIMBAL, 0, 0, 0, 0)
                    self.mav.mav.gimbal_device_information_send(
                        0, b"ArduPilot", b"Test", b"", 1, 1, component, 0, 0,
                        -math.pi, math.pi, -math.pi / 2, math.pi / 2, -math.pi, math.pi,
                        cap_flags2=mavutil.mavlink.GIMBAL_DEVICE_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL)
            finally:
                self.mav.mav.srcSystem, self.mav.mav.srcComponent = old_sysid, old_compid
            self.delay_sim_time(1, reason="discover test gimbals")

        here = self.get_location(frame=AltFrame.ABSOLUTE)
        lat, lon = int(here.lat * 1e7) + 1000, int(here.lng * 1e7)
        roi_command = mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION
        clear_command = mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE

        def roi(mount, offset=0, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
            self.run_cmd_int(roi_command, p1=mount, x=lat + offset, y=lon, z=600,
                             frame=mavutil.mavlink.MAV_FRAME_GLOBAL, want_result=want_result)

        def angle(mount, pitch):
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                         p1=pitch, p2=0, p3=float('nan'), p4=float('nan'), p7=mount)

        def check_targets(targets):
            # Discard commands in flight, then check a full refresh interval.
            self.delay_sim_time(0.4, reason="wait for mount target change")
            self.context_clear_collection('COMMAND_INT')
            self.context_clear_collection('GIMBAL_DEVICE_SET_ATTITUDE')
            self.delay_sim_time(0.6, reason="collect mount target refreshes")
            for component, (kind, value) in zip(components, targets):
                locations = [m for m in self.context_collection('COMMAND_INT')
                             if m.target_component == component and m.command == roi_command]
                angles = [m for m in self.context_collection('GIMBAL_DEVICE_SET_ATTITUDE')
                          if m.target_component == component]
                if kind == 'roi':
                    if angles or not locations or any(m.x != lat + value or m.y != lon or abs(m.z - 600) > 0.1
                                                      for m in locations):
                        raise NotAchievedException("Wrong ROI stream for gimbal %u" % component)
                else:
                    if locations or not angles:
                        raise NotAchievedException("Wrong angle stream for gimbal %u" % component)
                    if value is not None:
                        for msg in angles:
                            pitch = math.degrees(quaternion.Quaternion(msg.q).euler[1])
                            if abs(pitch - value) > 0.1:
                                raise NotAchievedException("Wrong pitch for gimbal %u" % component)
            for mount in (1, 2):
                if self.get_parameter("MNT%u_TARG_RATE" % mount) != 5:
                    raise NotAchievedException("ROI changed the mount target rate")

        angle(1, -10)
        angle(2, -20)
        roi(2, 2000)
        check_targets((('angle', -10), ('roi', 2000)))
        roi(1)
        check_targets((('roi', 0), ('roi', 2000)))
        angle(2, -30)
        check_targets((('roi', 0), ('angle', -30)))
        roi(2, 3000)
        self.context_clear_collection('COMMAND_INT')
        self.run_cmd(clear_command, p1=1)
        self.delay_sim_time(0.4, reason="wait for mount target change")
        cleared = [m.target_component for m in self.context_collection('COMMAND_INT') if m.command == clear_command]
        if not cleared or set(cleared) != {components[0]}:
            raise NotAchievedException("ROI_NONE was not sent only to mount 1")
        check_targets((('angle', None), ('roi', 3000)))
        angle(1, -15)
        self.run_cmd(clear_command, p1=2)
        check_targets((('angle', -15), ('angle', None)))

        # The updated selectors use actual, non-contiguous component IDs.
        roi(components[1], 4000)
        angle(components[0], -25)
        check_targets((('angle', -25), ('roi', 4000)))
        roi(components[0], 5000)
        angle(components[1], -35)
        check_targets((('roi', 5000), ('angle', -35)))
        self.run_cmd(clear_command, p1=components[0])
        angle(1, -15)
        self.run_cmd(clear_command, p1=2)
        for mount in (3, 153, 170, 255):
            roi(mount, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
            self.run_cmd(clear_command, p1=mount, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        for mount in (-1, 1.5, 256, float('nan'), float('inf')):
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                         p1=-70, p2=0, p7=mount, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        # Invalid selectors must not accidentally address a different mount.
        for mount in (-1, 1.5, 256, float('inf')):
            roi(mount, want_result=mavutil.mavlink.MAV_RESULT_DENIED)
            self.run_cmd(clear_command, p1=mount, want_result=mavutil.mavlink.MAV_RESULT_DENIED)
        check_targets((('angle', -15), ('angle', None)))
        # Legacy ROI uses param1 as a mode, and selector zero retains the
        # vehicle-wide behaviour. Neither may be reinterpreted as mount 3.
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_ROI, p1=mavutil.mavlink.MAV_ROI_LOCATION,
                         x=lat, y=lon, z=600, frame=mavutil.mavlink.MAV_FRAME_GLOBAL)
        check_targets((('roi', 0), ('angle', None)))
        self.run_cmd(clear_command)
        roi(0, 1000)
        check_targets((('roi', 1000), ('angle', None)))
        self.run_cmd(clear_command)
        roi(float('nan'), 1500)
        check_targets((('roi', 1500), ('angle', None)))
        self.run_cmd(clear_command, p1=float('nan'))
        check_targets((('angle', None), ('angle', None)))

        # Native discovery uses component IDs; numbered controls remain aliases.
        self.context_collect('GIMBAL_MANAGER_INFORMATION')
        self.send_poll_message('GIMBAL_MANAGER_INFORMATION')
        self.delay_sim_time(1, reason="collect selector replies")
        advertised = {m.gimbal_device_id for m in self.context_collection('GIMBAL_MANAGER_INFORMATION')}
        if advertised != set(components):
            raise NotAchievedException("Incorrect native gimbal IDs: %s" % advertised)

        # Cached native status keeps its source identity and its attitude frame.
        status_flags = mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME
        status_q = quaternion.Quaternion([0, math.radians(-20), math.radians(45)]).q

        def send_gimbal_status(mav, msg):
            if msg.get_type() != 'SYSTEM_TIME':
                return
            old_sysid, old_compid = self.mav.mav.srcSystem, self.mav.mav.srcComponent
            try:
                self.mav.mav.srcSystem = self.sysid_thismav()
                for component in components:
                    self.mav.mav.srcComponent = component
                    self.mav.mav.gimbal_device_attitude_status_send(
                        self.sysid_thismav(), 1, 1234, status_flags, status_q, 0, 0, 0, 0)
            finally:
                self.mav.mav.srcSystem, self.mav.mav.srcComponent = old_sysid, old_compid

        self.install_message_hook_context(send_gimbal_status)
        self.delay_sim_time(1, reason="provide native gimbal status")
        self.context_collect('GIMBAL_DEVICE_ATTITUDE_STATUS')
        self.send_poll_message('GIMBAL_DEVICE_ATTITUDE_STATUS')
        self.delay_sim_time(0.2, reason="collect cached native status")
        statuses = self.context_collection('GIMBAL_DEVICE_ATTITUDE_STATUS')
        if {m.get_srcComponent() for m in statuses} != set(components):
            raise NotAchievedException("Incorrect native gimbal status identities")
        for msg in statuses:
            if (msg.get_srcSystem() != self.sysid_thismav() or msg.gimbal_device_id != 0 or
                    msg.flags != status_flags or msg.time_boot_ms != 1234 or
                    msg.target_system != 0 or msg.target_component != 0 or
                    any(abs(a - b) > 1e-6 for a, b in zip(msg.q, status_q))):
                raise NotAchievedException("Native gimbal status was altered: %s" % msg)

        # Mission selectors use the same component IDs and numbered aliases.
        items = [self.mission_item_home(), self.mission_item_copter_takeoff(10)]
        items.extend([
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                                         p1=-40, p2=0, z=components[0]),
            self.create_MISSION_ITEM_INT(roi_command, p1=components[1], x=lat+6000, y=lon, z=600,
                                         frame=mavutil.mavlink.MAV_FRAME_GLOBAL),
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM),
            self.create_MISSION_ITEM_INT(clear_command, p1=2),
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM),
        ])
        for seq, item in enumerate(items):
            item.seq = seq
            item.autocontinue = 1
        self.check_mission_upload_download(items)
        self.takeoff(10, mode='GUIDED')
        self.change_mode('AUTO')
        self.wait_waypoint(4, 4)
        check_targets((('angle', -40), ('roi', 6000)))
        self.set_current_waypoint(5, check_afterwards=False)
        self.wait_waypoint(6, 6)
        # The DO command before the second loiter clears only the second mount.
        check_targets((('angle', -40), ('angle', None)))
        self.do_RTL()

        self.set_parameter("MNT2_TYPE", 0)
        self.reboot_sitl()
        roi(2, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.run_cmd(clear_command, p1=2, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
