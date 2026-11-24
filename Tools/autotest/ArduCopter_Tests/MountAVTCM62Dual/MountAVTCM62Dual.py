'''
Copter autotest: MountAVTCM62Dual

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase

from arducopter import CAMERA_IMAGE_STATUS_INTERVAL_IDLE
from arducopter import CAMERA_IMAGE_STATUS_IDLE


class MountAVTCM62Dual(TestGroup):
    def camera_settings_for_compid(self, compid):
        '''return most recently collected CAMERA_SETTINGS from compid, if any'''
        ret = None
        for m in self.context_collection('CAMERA_SETTINGS'):
            if m.get_srcComponent() == compid:
                ret = m
        return ret

    def wait_camera_settings(self, wants, timeout=30):
        '''wait for collected CAMERA_SETTINGS to show each (compid, zoom,
        focus) in wants; a wanted value of None is not checked.  The cameras
        emit CAMERA_SETTINGS in response to zoom and focus commands, so this
        returns as soon as those responses arrive'''
        tstart = self.get_sim_time()
        while True:
            unmet = []
            for (compid, want_zoom, want_focus) in wants:
                m = self.camera_settings_for_compid(compid)
                if m is None:
                    unmet.append("compid %u: no CAMERA_SETTINGS" % compid)
                    continue
                if want_zoom is not None and abs(m.zoomLevel - want_zoom) > 0.5:
                    unmet.append("compid %u: zoom want=%f got=%f" %
                                 (compid, want_zoom, m.zoomLevel))
                if want_focus is not None and abs(m.focusLevel - want_focus) > 0.5:
                    unmet.append("compid %u: focus want=%f got=%f" %
                                 (compid, want_focus, m.focusLevel))
            if len(unmet) == 0:
                return
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("; ".join(unmet))
            self.mav.recv_match(type='CAMERA_SETTINGS', blocking=True, timeout=0.1)

    def camera_feedback_img_idx(self, cam_idx):
        '''return most recently collected img_idx for cam_idx, or None.
        CAMERA_FEEDBACK is emitted for every camera whenever any camera
        triggers, so the most recent message from a camera carries that
        camera's current count of images taken since boot'''
        ret = None
        for m in self.context_collection('CAMERA_FEEDBACK'):
            if m.cam_idx == cam_idx:
                ret = m.img_idx
        return ret

    def wait_camera_img_idx(self, wants, timeout=30):
        '''wait for collected CAMERA_FEEDBACK to show each (cam_idx, img_idx)
        in wants'''
        tstart = self.get_sim_time()
        while True:
            unmet = []
            for (cam_idx, want_img_idx) in wants:
                got = self.camera_feedback_img_idx(cam_idx)
                if got != want_img_idx:
                    unmet.append("cam_idx %u: img_idx want=%s got=%s" %
                                 (cam_idx, want_img_idx, got))
            if len(unmet) == 0:
                return
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("; ".join(unmet))
            self.mav.recv_match(type='CAMERA_FEEDBACK', blocking=True, timeout=0.1)

    def _setup_avt_cm62_dual(self):
        '''configure two SIM_AVT_CM62 simulators on serial5 and serial6'''
        self.set_parameters({
            "MNT1_TYPE": 6,         # MAVLink
            "MNT2_TYPE": 6,         # MAVLink
            "CAM1_TYPE": 6,         # MAVLink
            "CAM2_TYPE": 6,         # MAVLink
            "SERIAL5_PROTOCOL": 2,  # MAVLink2
            "SERIAL6_PROTOCOL": 2,  # MAVLink2
        })
        self.customise_SITL_commandline([
            "--serial5=sim:avt_cm62_gimbal:",
            "--serial6=sim:avt_cm62_gimbal:",
        ])
        for instance in 1, 2:
            self.wait_camera_initialised(instance)
        self.context_collect('CAMERA_CAPTURE_STATUS')
        self.context_collect('CAMERA_FEEDBACK')
        self.context_collect('CAMERA_SETTINGS')

    @testcase(bucket="CopterTests2b")
    def MountAVTCM62Dual(self):
        '''test two simultaneous MAVLink (Gimbal Protocol v2) gimbals using
        two SIM_AVT_CM62 simulators on separate serial ports'''
        self._setup_avt_cm62_dual()

        # note that CAMERA_FEEDBACK uses instance numbers starting from 0
        want_img_idx = [1, 1]
        self.progress("Test triggering of the two cameras")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_SET_CAM_TRIGG_DIST,
            p1=10,  # trigger distance
            p3=1,   # trigger instantly
            p4=0,   # 0 means trigger for all cameras
        )
        self.wait_camera_img_idx(list(enumerate(want_img_idx)))

        for cam in 0, 1:
            self.progress(f"Test triggering of the just cam{cam}")
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_SET_CAM_TRIGG_DIST,
                p1=10,  # trigger distance
                p3=1,   # trigger instantly
                p4=cam+1,
            )
            # the count for the other camera must not move
            want_img_idx[cam] += 1
            self.wait_camera_img_idx(list(enumerate(want_img_idx)))

        cam1_compid = mavutil.mavlink.MAV_COMP_ID_CAMERA
        cam2_compid = mavutil.mavlink.MAV_COMP_ID_CAMERA + 1  # MAV_COMP_ID_CAMERA2

        self.progress("Test setting different zoom levels on each camera")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM,
            p1=mavutil.mavlink.ZOOM_TYPE_RANGE,
            p2=30,  # 30%
            p3=1,   # camera instance 1
        )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM,
            p1=mavutil.mavlink.ZOOM_TYPE_RANGE,
            p2=70,  # 70%
            p3=2,   # camera instance 2
        )
        self.wait_camera_settings([
            (cam1_compid, 30, None),
            (cam2_compid, 70, None),
        ])

        self.progress("Test setting different focus levels on each camera")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS,
            p1=mavutil.mavlink.FOCUS_TYPE_RANGE,
            p2=20,  # 20%
            p3=1,   # camera instance 1
        )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS,
            p1=mavutil.mavlink.FOCUS_TYPE_RANGE,
            p2=80,  # 80%
            p3=2,   # camera instance 2
        )
        # the zoom levels set above must be unchanged by the focus commands
        self.wait_camera_settings([
            (cam1_compid, 30, 20),
            (cam2_compid, 70, 80),
        ])

        self.progress("Test setting zoom and focus on all cameras at once")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM,
            p1=mavutil.mavlink.ZOOM_TYPE_RANGE,
            p2=50,  # 50%
            p3=0,   # 0 means all cameras
        )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS,
            p1=mavutil.mavlink.FOCUS_TYPE_RANGE,
            p2=60,  # 60%
            p3=0,   # 0 means all cameras
        )
        self.wait_camera_settings([
            (cam1_compid, 50, 60),
            (cam2_compid, 50, 60),
        ])

    @testcase(bucket="CopterTests2b")
    def MountAVTCM62DualMission(self):
        '''test dual AVT CM62 cameras with photo capture, zoom and focus
        controlled via mission items'''
        self._setup_avt_cm62_dual()

        cam1_compid = mavutil.mavlink.MAV_COMP_ID_CAMERA
        cam2_compid = mavutil.mavlink.MAV_COMP_ID_CAMERA + 1  # MAV_COMP_ID_CAMERA2

        self.set_parameter("AUTO_OPTIONS", 3)

        mission_items = [
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
                p1=0,    # 0 means all cameras
                p2=0,    # interval (0 = single shot)
                p3=1,    # total images = 1
                autocontinue=1,
            ),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
                p1=1,    # camera instance 1
                p2=0,    # interval (0 = single shot)
                p3=1,    # total images = 1
                autocontinue=1,
            ),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
                p1=2,    # camera instance 2
                p2=1,    # interval (s)
                p3=2,    # total images = 2
                autocontinue=1,
            ),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM,
                p1=mavutil.mavlink.ZOOM_TYPE_RANGE,
                p2=25,   # cam1 → 25%
                p3=1,    # camera instance 1
                autocontinue=1,
            ),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM,
                p1=mavutil.mavlink.ZOOM_TYPE_RANGE,
                p2=75,   # cam2 → 75%
                p3=2,    # camera instance 2
                autocontinue=1,
            ),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS,
                p1=mavutil.mavlink.FOCUS_TYPE_RANGE,
                p2=40,   # cam1 → 40%
                p3=1,    # camera instance 1
                autocontinue=1,
            ),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS,
                p1=mavutil.mavlink.FOCUS_TYPE_RANGE,
                p2=80,   # cam2 → 80%
                p3=2,    # camera instance 2
                autocontinue=1,
            ),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
                p1=1,    # camera instance 1
                p2=1,    # interval (s)
                p3=0,    # total images = 0 (capture until stopped)
                autocontinue=1,
            ),
            # the do-commands above all run while the takeoff is the current
            # mission item, so camera 1 captures for the duration of the
            # climb; the stop below runs once the waypoint becomes current
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 30, 0, 20),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE,
                p1=1,    # camera instance 1
                autocontinue=1,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ]

        self.start_flying_simple_relhome_mission(mission_items)

        self.progress("Verify CAMERA_CAPTURE_STATUS reports the interval capture")
        # wait for camera 2 to finish the images it was asked for (one
        # from the all-cameras item, two from its own item), leaving
        # camera 1 as the only one with an interval set for the rest of
        # the climb
        self.wait_camera_img_idx([(1, 3)])
        got = sorted(self.camera_capture_statuses(2))
        if got != [CAMERA_IMAGE_STATUS_IDLE, CAMERA_IMAGE_STATUS_INTERVAL_IDLE]:
            raise NotAchievedException(
                f"Wanted exactly one camera capturing on an interval: {got}")

        # the return-to-launch is the last item, so home is its sequence
        # number; by the time it is current camera 1 has been stopped
        self.wait_current_waypoint(len(mission_items))
        self.progress("Verify camera 1 captured images until stopped")
        img_idx_at_stop = self.camera_feedback_img_idx(0)
        if img_idx_at_stop is None or img_idx_at_stop <= 1:
            raise NotAchievedException(
                f"cam_idx 0 did not capture until stopped: got={img_idx_at_stop}")

        # the descent and landing which follow give camera 1 plenty of time
        # to take more images if IMAGE_STOP_CAPTURE did not stop it
        self.wait_disarmed()

        self.progress("Verify per-camera shot counts from mission items")
        self.wait_camera_img_idx([(0, img_idx_at_stop), (1, 3)])

        self.progress("Verify CAMERA_CAPTURE_STATUS reports no interval capture")
        got = self.camera_capture_statuses(2)
        if got != [CAMERA_IMAGE_STATUS_IDLE, CAMERA_IMAGE_STATUS_IDLE]:
            raise NotAchievedException(f"Camera still has an interval set: {got}")

        self.progress("Verify per-camera zoom and focus set by mission items")
        self.wait_camera_settings([
            (cam1_compid, 25, 40),
            (cam2_compid, 75, 80),
        ])

    @testcase(bucket="CopterTests2b")
    def MountAVTCM62DualImageStartCapture(self):
        '''test MAV_CMD_IMAGE_START_CAPTURE mavlink command against two
        SIM_AVT_CM62 cameras'''
        self._setup_avt_cm62_dual()

        # note that CAMERA_FEEDBACK uses cam_idx numbers starting from 0
        want_img_idx = [1, 1]
        self.progress("Single image on all cameras (interval and total both zero)")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
            p1=0,  # 0 means all cameras
            p2=0,  # interval
            p3=0,  # total images; zero with a zero interval means one image
        )
        self.wait_camera_img_idx(list(enumerate(want_img_idx)))

        for cam in 0, 1:
            self.progress(f"Single image on just cam{cam} (total images is 1)")
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
                p1=cam+1,  # camera instance
                p2=0,      # interval
                p3=1,      # total images
            )
            # the count for the other camera must not move
            want_img_idx[cam] += 1
            self.wait_camera_img_idx(list(enumerate(want_img_idx)))

        self.progress("Fixed number of images on just cam0, via COMMAND_LONG")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
            p1=1,  # camera instance 1
            p2=1,  # interval (s)
            p3=3,  # total images
        )
        want_img_idx[0] += 3
        self.wait_camera_img_idx(list(enumerate(want_img_idx)))

        self.progress("Capture-until-stopped on cam1 (total images is 0)")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
            p1=2,  # camera instance 2
            p2=1,  # interval (s)
            p3=0,  # total images; zero with an interval means until stopped
        )
        # wait for several images so we know it does not stop by itself
        want_img_idx[1] += 3
        self.wait_camera_img_idx(list(enumerate(want_img_idx)))
        got = sorted(self.camera_capture_statuses(2))
        if got != [CAMERA_IMAGE_STATUS_IDLE, CAMERA_IMAGE_STATUS_INTERVAL_IDLE]:
            raise NotAchievedException(
                f"Wanted exactly one camera capturing on an interval: {got}")

        self.progress("Stop the capture on cam1")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE,
            p1=2,  # camera instance 2
        )
        self.progress("Wait for CAMERA_CAPTURE_STATUS to report no interval capture")
        tstart = self.get_sim_time()
        while True:
            got = self.camera_capture_statuses(2)
            if got == [CAMERA_IMAGE_STATUS_IDLE, CAMERA_IMAGE_STATUS_IDLE]:
                break
            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException(
                    f"Camera still has an interval set: {got}")
        img_idx_at_stop = self.camera_feedback_img_idx(1)

        self.progress("Multiple images with a zero interval must be denied")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
            p1=1,  # camera instance 1
            p2=0,  # interval
            p3=2,  # total images
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )

        self.progress("Negative camera id must be denied")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
            p1=-1,  # invalid camera instance
            p2=0,   # interval
            p3=1,   # total images
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )

        self.progress("Capture on an absent camera instance must be denied")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
            p1=3,  # camera instance 3 does not exist
            p2=0,  # interval
            p3=1,  # total images
            want_result=mavutil.mavlink.MAV_RESULT_DENIED,
        )

        self.progress("Two-image capture on cam0; cam1 must remain stopped")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
            p1=1,  # camera instance 1
            p2=1,  # interval (s)
            p3=2,  # total images
        )
        # waiting for these images to arrive also spans two of cam1's
        # old capture intervals, so this checks cam1 really stopped and
        # that the denied/failed commands above took no images
        want_img_idx[0] += 2
        self.wait_camera_img_idx([(0, want_img_idx[0]), (1, img_idx_at_stop)])
