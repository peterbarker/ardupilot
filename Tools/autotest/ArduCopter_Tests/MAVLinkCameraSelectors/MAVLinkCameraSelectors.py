'''
Copter autotest: MAVLinkCameraSelectors

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAVLinkCameraSelectors(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MAVLinkCameraSelectors(self):
        """select cameras by component ID or legacy slot without confusing stream IDs"""
        self.set_parameters({"CAM1_TYPE": 6, "CAM2_TYPE": 6,
                             "CAM1_COMPID": 105, "CAM2_COMPID": 100,
                             "SERIAL1_PROTOCOL": 2, "SERIAL2_PROTOCOL": 2})
        self.reboot_sitl()
        saved_mavfile_global = mavutil.mavfile_global
        cameras = []
        try:
            for serial, component in ((1, 105), (2, 100)):
                cameras.append(mavutil.mavlink_connection(
                    self.sitl_serial_endpoint(serial), source_system=self.sysid_thismav(), source_component=component))
            mavutil.mavfile_global = saved_mavfile_global
            flags = (mavutil.mavlink.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                     mavutil.mavlink.CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                     mavutil.mavlink.CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM |
                     mavutil.mavlink.CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS)
            for _ in range(15):
                for camera in cameras:
                    camera.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_CAMERA, 0, 0, 0, 0)
                    camera.mav.camera_information_send(
                        0, b"ArduPilot".ljust(32, b'\0'), b"Selectors".ljust(32, b'\0'),
                        1, 0, 0, 0, 1920, 1080, 0, flags, 0, b"")
                self.delay_sim_time(1, reason="discover cameras for selector tests")

            def received_commands():
                result = []
                for index, camera in enumerate(cameras):
                    while (msg := camera.recv_match()) is not None:
                        if (msg.get_type() == 'COMMAND_LONG' and
                                msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1 and
                                msg.target_component == camera.mav.srcComponent):
                            result.append((index, msg))
                return result

            def check(command, params, expected, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                      downstream_params=None, command_int=False):
                received_commands()
                if command_int:
                    self.run_cmd_int(command, want_result=want_result, **params)
                else:
                    self.run_cmd(command, want_result=want_result, **params)
                self.delay_sim_time(0.5, reason="collect selected camera commands")
                commands = [(index, msg) for index, msg in received_commands() if msg.command == command]
                if sorted(index for index, _ in commands) != sorted(expected):
                    raise NotAchievedException("Camera selector routed incorrectly: %s" % commands)
                for _, msg in commands:
                    for name, value in (downstream_params or {}).items():
                        if abs(getattr(msg, name) - value) > 0.001:
                            raise NotAchievedException("Camera stream parameters changed: %s" % msg)

            image = mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE
            zoom = mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM
            focus = mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS
            start = mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE
            stop = mavutil.mavlink.MAV_CMD_VIDEO_STOP_CAPTURE
            denied = mavutil.mavlink.MAV_RESULT_DENIED
            for selector, index in ((1, 0), (2, 1), (105, 0), (100, 1)):
                check(image, {'p1': selector, 'p3': 1}, [index])
                check(image, {'p1': selector, 'p3': 1}, [index], command_int=True)
                check(zoom, {'p1': 2, 'p2': 50, 'p3': selector}, [index])
                check(focus, {'p1': 1, 'p2': 50, 'p3': selector}, [index])
                check(start, {'p1': 3, 'p2': 2, 'p3': selector}, [index],
                      downstream_params={'param1': 3, 'param2': 2, 'param3': 0})
                check(stop, {'p1': 3, 'p2': selector}, [index],
                      downstream_params={'param1': 3, 'param2': 0})
            # Preserve old stream-as-camera and default-primary recording.
            for selector, index in ((0, 0), (1, 0), (2, 1)):
                check(start, {'p1': selector}, [index], downstream_params={'param1': 0})
                check(stop, {'p1': selector}, [index], downstream_params={'param1': 0})
            # Old XML specified NaN for the now-repurposed video selectors.
            check(start, {'p1': 2, 'p3': float('nan')}, [1], downstream_params={'param1': 0})
            check(stop, {'p1': 2, 'p2': float('nan')}, [1], downstream_params={'param1': 0})
            check(zoom, {'p1': 2, 'p2': 50, 'p3': float('nan')}, [0, 1])
            check(focus, {'p1': 1, 'p2': 50, 'p3': float('nan')}, [0, 1])
            check(image, {'p1': float('nan'), 'p3': 1}, [], denied)
            check(image, {'p3': 1}, [0, 1])
            for selector in (-1, 1.5, 3, 99, 101, 256, float('inf')):
                check(image, {'p1': selector, 'p3': 1}, [], denied)
                check(zoom, {'p1': 2, 'p2': 50, 'p3': selector}, [], denied)
                check(start, {'p1': 1, 'p3': selector}, [], denied)
            for stream in (-1, 1.5, 256, float('nan'), float('inf')):
                check(start, {'p1': stream, 'p3': 100}, [], denied)
            # Reject unknown camera selectors even for unsupported operations;
            # a fractional COMMAND_LONG param5 must not truncate to camera 1.
            for command, params in (
                    (mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT, {'p4': 99}),
                    (mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE, {'p5': 1.5}),
                    (mavutil.mavlink.MAV_CMD_CAMERA_STOP_TRACKING, {'p1': 99})):
                check(command, params, [], denied)
            check(mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE, {'p5': float('nan')}, [],
                  mavutil.mavlink.MAV_RESULT_UNSUPPORTED)

            # Exercise explicit selectors on isolated links as well as normal links.
            self.set_parameters({"MAV2_OPTIONS": 16, "MAV3_OPTIONS": 2})
            check(start, {'p1': 2, 'p3': 105}, [0], downstream_params={'param1': 2})
            check(stop, {'p1': 2, 'p2': 100}, [1], downstream_params={'param1': 2})

            # Mission storage and execution must use the same selectors as live commands.
            items = [self.mission_item_home(), self.mission_item_copter_takeoff(10)]
            for command, params in (
                    (start, {'p1': 3, 'p2': 2, 'p3': 100}),
                    (image, {'p1': 105, 'p3': 1}),
                    (zoom, {'p1': 2, 'p2': 42, 'p3': 100}),
                    (focus, {'p1': 1, 'p2': 50, 'p3': 105}),
                    (stop, {'p1': 3, 'p2': 100})):
                items.append(self.create_MISSION_ITEM_INT(command, **params))
            items.append(self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM))
            for seq, item in enumerate(items):
                item.seq = seq
                item.autocontinue = 1
            self.check_mission_upload_download(items)
            self.takeoff(10, mode='GUIDED')
            received_commands()
            self.change_mode('AUTO')
            self.wait_waypoint(len(items) - 1, len(items) - 1)
            self.delay_sim_time(1, reason="collect selector replies")
            commands = [(index, msg) for index, msg in received_commands()
                        if msg.command in (start, stop, image, zoom, focus)]
            actual = [(index, msg.command) for index, msg in commands]
            expected = [(1, start), (0, image), (1, zoom), (0, focus), (1, stop)]
            if sorted(actual) != sorted(expected):
                raise NotAchievedException("Mission camera routing: %s" % commands)
            video = next(msg for _, msg in commands if msg.command == start)
            if video.param1 != 3 or video.param2 != 2 or video.param3 != 0:
                raise NotAchievedException("Mission video parameters lost: %s" % video)
            self.do_RTL()
        finally:
            mavutil.mavfile_global = saved_mavfile_global
            for camera in cameras:
                camera.close()
