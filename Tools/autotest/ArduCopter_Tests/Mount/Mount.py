'''
Copter autotest: Mount

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra
from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class Mount(TestGroup):
    def do_pitch(self, pitch):
        '''pitch aircraft in guided/angle mode'''
        self.mav.mav.set_attitude_target_send(
            0, # time_boot_ms
            1, # target sysid
            1, # target compid
            0, # bitmask of things to ignore
            mavextra.euler_to_quat([0, math.radians(pitch), 0]), # att
            0, # roll rate  (rad/s)
            0, # pitch rate (rad/s)
            0, # yaw rate   (rad/s)
            0.5) # thrust, 0 to 1, translated to a climb/descent rate

    def mount_test_body(self, pitch_rc_neutral=1500, do_rate_tests=True, constrain_sysid_target=True, neutral_tol_deg=0,
                        rc_targetting_pitch_tolerance=0.1):
        '''Test Camera/Antenna Mount - assumes a camera is set up and ready to go'''
        if True:
            # make sure we're getting gimbal device attitude status
            self.assert_receive_message('GIMBAL_DEVICE_ATTITUDE_STATUS', timeout=5, very_verbose=True)

            # change mount to neutral mode (point forward, not stabilising)
            self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)

            # wait for mount to report neutral angles (external MAVLink gimbals may
            # have in-flight messages from the previous mode in the receive queue)
            tstart = self.get_sim_time()
            while True:
                mount_roll_deg, mount_pitch_deg, mount_yaw_deg, mount_yaw_is_absolute = self.get_mount_roll_pitch_yaw_deg()
                if (abs(mount_roll_deg) <= neutral_tol_deg and
                        abs(mount_pitch_deg) <= neutral_tol_deg and
                        abs(mount_yaw_deg) <= neutral_tol_deg):
                    break
                if self.get_sim_time() - tstart > 5:
                    raise NotAchievedException("Mount not neutral (r=%f p=%f y=%f)" % (
                        mount_roll_deg, mount_pitch_deg, mount_yaw_deg))

            self.takeoff(30, mode='GUIDED')

            # pitch vehicle back and confirm gimbal is still not stabilising
            despitch = 10
            despitch_tolerance = 3

            self.progress("Pitching vehicle")
            self.do_pitch(despitch) # will time out!

            self.wait_pitch(despitch, despitch_tolerance)

            # check gimbal is still not stabilising
            mount_roll_deg, mount_pitch_deg, mount_yaw_deg, mount_yaw_is_absolute = self.get_mount_roll_pitch_yaw_deg()
            if (abs(mount_roll_deg) > neutral_tol_deg or
                    abs(mount_pitch_deg) > neutral_tol_deg or
                    abs(mount_yaw_deg) > neutral_tol_deg):
                raise NotAchievedException("Mount stabilising when not requested")

            # center RC tilt control and change mount to RC_TARGETING mode
            self.progress("Gimbal to RC Targeting mode")
            self.set_rc(6, pitch_rc_neutral)
            self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)

            # pitch vehicle back and confirm gimbal is stabilising
            self.progress("Pitching vehicle")
            self.do_pitch(despitch)
            self.wait_pitch(despitch, despitch_tolerance)
            self.test_mount_pitch(0, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)

            # point gimbal at specified angle
            self.progress("Point gimbal using GIMBAL_MANAGER_PITCHYAW (ANGLE)")
            self.do_pitch(0)    # level vehicle
            self.wait_pitch(0, despitch_tolerance)
            self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
            for (method, angle) in (self.run_cmd, -20), (self.run_cmd_int, -30):
                method(
                    mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                    p1=angle,   # pitch angle in degrees
                    p2=0,     # yaw angle in degrees
                    p3=0,     # pitch rate in degrees (NaN to ignore)
                    p4=0,     # yaw rate in degrees (NaN to ignore)
                    p5=0,     # flags (0=Body-frame, 16/GIMBAL_MANAGER_FLAGS_YAW_LOCK=Earth Frame)
                    p6=0,     # unused
                    p7=0,     # gimbal id
                )
                self.test_mount_pitch(angle, 1, mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)

            # this is a one-off; ArduCopter *will* time out this directive!
            self.progress("Levelling aircraft")
            self.mav.mav.set_attitude_target_send(
                0, # time_boot_ms
                1, # target sysid
                1, # target compid
                0, # bitmask of things to ignore
                mavextra.euler_to_quat([0, 0, 0]), # att
                0, # roll rate  (rad/s)
                0, # pitch rate (rad/s)
                0, # yaw rate   (rad/s)
                0.5) # thrust, 0 to 1, translated to a climb/descent rate

            self.wait_groundspeed(0, 1)

            # now test RC targeting
            self.progress("Testing mount RC targeting")

            self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
            self.test_mount_rc_targetting(
                pitch_rc_neutral=pitch_rc_neutral,
                do_rate_tests=do_rate_tests,
                pitch_tolerance=rc_targetting_pitch_tolerance,
            )

            self.progress("Testing mount ROI behaviour")
            self.test_mount_pitch(0, rc_targetting_pitch_tolerance, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
            start = self.get_location()
            self.progress("start=%s" % str(start))
            (roi_lat, roi_lon) = mavextra.gps_offset(start.lat,
                                                     start.lng,
                                                     10,
                                                     20)
            roi_alt = 0
            self.progress("Using MAV_CMD_DO_SET_ROI_LOCATION")
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                p5=roi_lat,
                p6=roi_lon,
                p7=roi_alt,
            )
            self.test_mount_pitch(-52, 5, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT)
            self.progress("Using MAV_CMD_DO_SET_ROI_LOCATION")
            # start by pointing the gimbal elsewhere with a
            # known-working command:
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                p5=roi_lat + 1,
                p6=roi_lon + 1,
                p7=roi_alt,
            )
            # now point it with command_int:
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                p5=int(roi_lat * 1e7),
                p6=int(roi_lon * 1e7),
                p7=roi_alt,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            )
            self.test_mount_pitch(-52, 5, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT)

            self.progress("Using MAV_CMD_DO_SET_ROI_NONE")
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE)
            self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE)
            self.test_mount_pitch(0, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)

            start = self.get_location()
            (roi_lat, roi_lon) = mavextra.gps_offset(start.lat,
                                                     start.lng,
                                                     -100,
                                                     -200)
            roi_alt = 0
            self.progress("Using MAV_CMD_DO_SET_ROI")
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI,
                p5=roi_lat,
                p6=roi_lon,
                p7=roi_alt,
            )
            self.test_mount_pitch(-7.5, 1, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT)

            start = self.get_location()
            (roi_lat, roi_lon) = mavextra.gps_offset(start.lat,
                                                     start.lng,
                                                     -100,
                                                     -200)
            roi_alt = 0
            self.progress("Using MAV_CMD_DO_SET_ROI (COMMAND_INT)")
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI,
                0,
                0,
                0,
                0,
                int(roi_lat*1e7),
                int(roi_lon*1e7),
                roi_alt,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            )
            self.test_mount_pitch(-7.5, 1, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT)
            self.progress("Using MAV_CMD_DO_SET_ROI (COMMAND_INT), absolute-alt-frame")
            # this is pointing essentially straight down
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI,
                0,
                0,
                0,
                0,
                int(roi_lat*1e7),
                int(roi_lon*1e7),
                roi_alt,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
            )
            self.test_mount_pitch(-70, 1, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT, hold=2)

            self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)
            self.test_mount_pitch(0, 0.1, mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)

            self.progress("Testing mount roi-sysid behaviour")
            self.test_mount_pitch(0, 0.1, mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)
            start = self.get_location()
            self.progress("start=%s" % str(start))
            (roi_lat, roi_lon) = mavextra.gps_offset(start.lat,
                                                     start.lng,
                                                     10,
                                                     20)
            roi_alt = 0
            self.progress("Using MAV_CMD_DO_SET_ROI_SYSID")
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI_SYSID,
                p1=self.mav.source_system,
            )
            self.mav.mav.global_position_int_send(
                0, # time boot ms
                int(roi_lat * 1e7),
                int(roi_lon * 1e7),
                0 * 1000, # mm alt amsl
                0 * 1000, # relalt mm UP!
                0, # vx
                0, # vy
                0, # vz
                0 # heading
            )
            self.test_mount_pitch(-89, 5, mavutil.mavlink.MAV_MOUNT_MODE_SYSID_TARGET, hold=2)

            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE)
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI_SYSID,
                p1=self.mav.source_system,
            )
            self.mav.mav.global_position_int_send(
                0, # time boot ms
                int(roi_lat * 1e7),
                int(roi_lon * 1e7),
                670 * 1000, # mm alt amsl
                100 * 1000, # mm UP!
                0, # vx
                0, # vy
                0, # vz
                0 # heading
            )
            self.test_mount_pitch(
                68,
                5,
                mavutil.mavlink.MAV_MOUNT_MODE_SYSID_TARGET,
                hold=2,
                constrained=constrain_sysid_target,
            )

            self.progress("Testing mount holds last angle once sysid target telemetry goes stale")
            pre_stale_pitch = self.get_mount_roll_pitch_yaw_deg()[1]
            self.delay_sim_time(3.5, reason="let sysid target telemetry go stale")
            # move a long way in a direction that would swing the
            # elevation angle a lot if the mount were still (incorrectly)
            # tracking the stale target location as we move; it should
            # instead hold the angle last commanded above
            startpos = self.assert_receive_message('LOCAL_POSITION_NED')
            orig_x, orig_y, orig_z = startpos.x, startpos.y, startpos.z
            self.fly_guided_move_local(orig_x, orig_y + 80, -orig_z, timeout=60)
            self.test_mount_pitch(
                pre_stale_pitch,
                3,
                mavutil.mavlink.MAV_MOUNT_MODE_SYSID_TARGET,
                timeout=5,
                hold=2,
                constrained=False,
            )

            # move back to the original position, then confirm fresh
            # telemetry resumes tracking immediately (ie. not stuck forever)
            self.fly_guided_move_local(orig_x, orig_y, -orig_z, timeout=60)
            self.mav.mav.global_position_int_send(
                0, # time boot ms
                int(roi_lat * 1e7),
                int(roi_lon * 1e7),
                670 * 1000, # mm alt amsl
                100 * 1000, # mm UP!
                0, # vx
                0, # vy
                0, # vz
                0 # heading
            )
            self.test_mount_pitch(
                68, 5, mavutil.mavlink.MAV_MOUNT_MODE_SYSID_TARGET, hold=1,
                constrained=constrain_sysid_target,
            )

            self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)
            self.test_mount_pitch(0, 0.1, mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)

            # land so the body-yaw subtest takes off from the ground.
            # After a force-disarm in the air it re-armed shortly after
            # the falling vehicle hit the ground, and polling
            # change_mode() and armed() moved that re-arm closer still
            # to the impact; the vehicle then never got home:
            #     Mount/MountTopotek: "Did not get home"
            self.land_and_disarm()

            self.test_mount_body_yaw()

    def test_mount_body_yaw(self):
        '''check reporting of yaw'''
        # change mount to neutral mode (point forward, not stabilising)
        self.takeoff(10, mode='GUIDED')

        self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)

        for heading in 30, 45, 150:
            self.guided_achieve_heading(heading, direction=1, accuracy=1)

            r, p , y, yaw_is_absolute = self.get_mount_roll_pitch_yaw_deg()

            if yaw_is_absolute:
                raise NotAchievedException("Expected a relative yaw")

            if y > 1:
                raise NotAchievedException("Bad yaw (y=%f)")

        self.do_RTL()

    def assert_mount_retracted(self, retracted):
        '''assert whether GIMBAL_DEVICE_ATTITUDE_STATUS reports the mount retracted'''
        self.drain_mav()
        m = self.assert_receive_message('GIMBAL_DEVICE_ATTITUDE_STATUS', timeout=5)
        is_retracted = bool(m.flags & mavutil.mavlink.GIMBAL_DEVICE_FLAGS_RETRACT)
        if is_retracted != retracted:
            raise NotAchievedException(f"Expected retracted={retracted} got flags={m.flags}")

    def test_mount_configure_invalid_modes(self):
        '''check DO_MOUNT_CONFIGURE denies invalid modes without changing mode'''
        self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)
        self.assert_mount_retracted(False)
        # 256 would truncate to RETRACT if converted to uint8_t unchecked
        for mode in 8, 255, 256, -1, float('nan'), float('inf'), float('-inf'):
            for run_cmd in self.run_cmd, self.run_cmd_int:
                run_cmd(
                    mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
                    p1=mode,
                    want_result=mavutil.mavlink.MAV_RESULT_DENIED,
                )
        self.assert_mount_retracted(False)
        # make sure the check above can detect a retracted mount
        self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_RETRACT)
        self.assert_mount_retracted(True)
        self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING)

    @testcase(bucket="CopterTests2a")
    def Mount(self):
        '''test servo mount'''
        self.setup_servo_mount()
        self.reboot_sitl() # to handle MNT_TYPE changing
        self.test_mount_configure_invalid_modes()
        self.mount_test_body()

    @testcase(bucket="CopterTests2d")
    def MountSolo(self):
        '''test type=2, a "Solo" mount'''
        self.set_parameters({
            "MNT1_TYPE": 2,
            "RC6_OPTION": 213,  # MOUNT1_PITCH
        })
        self.customise_SITL_commandline([
            "--gimbal" # connects on port 5762
        ])
        self.mount_test_body(
            pitch_rc_neutral=1818,
            do_rate_tests=False,  # solo can't do rate control (yet?)
            constrain_sysid_target=False,  # not everything constrains all angles
        )

    def mount_check_camera_information(
            self,
            expected_vendor: str,
            expected_model: str,
            expected_fw_version: int | None = None,
            expected_cap_flags: int | None = None,
            expected_gimbal_device_id: int | None = None,
            timeout: int = 30,
    ) -> None:
        '''poll CAMERA_INFORMATION, retrying until vendor name, model name and
        firmware version all match what the simulated backend sends.  Vendor
        name is used to detect driver initialisation; model name and firmware
        version may arrive in separate handshake messages so we keep retrying
        on empty/zero values rather than failing immediately.'''
        self.assert_receive_message('GIMBAL_DEVICE_ATTITUDE_STATUS', timeout=10, very_verbose=True)
        # param2 of MAV_CMD_REQUEST_MESSAGE selects a specific camera instance
        # (1-based); 0 requests all cameras.  Use the gimbal_device_id directly
        # as the selector so each call retrieves exactly one camera's data.
        poll_p2 = expected_gimbal_device_id if expected_gimbal_device_id is not None else 0
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException(
                    "Never received complete CAMERA_INFORMATION for vendor=%s" % expected_vendor)
            try:
                m = self.poll_message('CAMERA_INFORMATION', timeout=5, p2=poll_p2)
            except NotAchievedException:
                continue
            got_vendor = bytes(m.vendor_name).split(b'\x00')[0].decode('utf-8')
            if not got_vendor:
                continue  # driver not initialised yet
            if got_vendor != expected_vendor:
                raise NotAchievedException(
                    "Unexpected vendor name: want=%s got=%s" % (expected_vendor, got_vendor))
            got_model = bytes(m.model_name).split(b'\x00')[0].decode('utf-8')
            if not got_model:
                continue  # model name handshake not complete yet
            if got_model != expected_model:
                raise NotAchievedException(
                    "Unexpected model name: want=%s got=%s" % (expected_model, got_model))
            if expected_fw_version is not None:
                if m.firmware_version == 0:
                    continue  # firmware version handshake not complete yet
                if m.firmware_version != expected_fw_version:
                    raise NotAchievedException(
                        "Unexpected firmware version: want=0x%08x got=0x%08x" %
                        (expected_fw_version, m.firmware_version))
            if expected_cap_flags is not None:
                if m.flags != expected_cap_flags:
                    raise NotAchievedException(
                        "Unexpected cap flags: want=0x%08x got=0x%08x" %
                        (expected_cap_flags, m.flags))
            return

    @testcase(bucket="CopterTests1e")
    def MountSiyiZT30(self):
        '''test Siyi ZT30 gimbal using SIM_Siyi_ZT30 simulator'''
        self.set_parameters({
            "MNT1_TYPE": 8,       # Siyi
            "CAM1_TYPE": 4,       # Mount
            "SERIAL5_PROTOCOL": 8,  # gimbal
            "RC6_OPTION": 213,    # MOUNT1_PITCH
        })
        self.customise_SITL_commandline(["--serial5=sim:siyi_zt30:"])
        # camera fw 1.2.3 from SIM_Siyi_ZT30: major=1, minor=2, patch=3
        # firmware_version = major | (minor<<8) | (patch<<16) = 0x030201
        # cap flags: CAPTURE_VIDEO | CAPTURE_IMAGE | HAS_BASIC_ZOOM | HAS_BASIC_FOCUS
        self.mount_check_camera_information(
            "Siyi", "ZT30",
            expected_fw_version=0x030201,
            expected_cap_flags=0xC3,
        )

        # Camera slot 2 can use mount slot 1; the two IDs must stay distinct.
        self.set_parameters({"CAM1_TYPE": 0, "CAM2_TYPE": 4, "CAM2_MNT_INST": 1})
        self.reboot_sitl()
        self.mount_check_camera_information("Siyi", "ZT30")
        info = self.poll_message('CAMERA_INFORMATION')
        if info.camera_device_id != 2 or info.gimbal_device_id != 1:
            raise NotAchievedException("Mount-backed camera IDs incorrect: %s" % info)
        settings = self.poll_message('CAMERA_SETTINGS')
        if settings.camera_device_id != 2:
            raise NotAchievedException("Mount-backed settings use the mount ID: %s" % settings)

    @testcase(bucket="CopterTests1c")
    def MountTopotek(self):
        '''test Topotek gimbal using SIM_Topotek simulator'''
        self.set_parameters({
            "MNT1_TYPE": 12,      # Topotek
            "CAM1_TYPE": 4,       # Mount
            "SERIAL5_PROTOCOL": 8,  # gimbal
            "RC6_OPTION": 213,    # MOUNT1_PITCH
        })
        self.customise_SITL_commandline(["--serial5=sim:topotek:"])
        # version "1.0.0" from SIM_Topotek: major=1 | (minor=0)<<8 | (patch=0)<<16 = 1
        # cap flags: CAPTURE_VIDEO | CAPTURE_IMAGE | HAS_BASIC_ZOOM | HAS_BASIC_FOCUS |
        #            HAS_TRACKING_POINT | HAS_TRACKING_RECTANGLE
        self.mount_check_camera_information(
            "Topotek", "SIM_TP",
            expected_fw_version=1,
            expected_cap_flags=0x6C3,
        )
        # pitch_rc_neutral=1818: with RC6 min=1000 max=2000 trim=1500 and
        # default MNT1_PITCH_MIN=-90 / MNT1_PITCH_MAX=20, norm_input=0.636
        # maps to exactly 0 deg pitch.
        # constrain_sysid_target=False: the driver does not yet clamp pitch to
        # MNT1_PITCH_MAX before sending, so the 68-deg sysid test is skipped
        # until AP_Mount_Topotek::send_target_angles enforces the limit.
        self.mount_test_body(pitch_rc_neutral=1818, do_rate_tests=False,
                             constrain_sysid_target=False)

    @testcase(bucket="CopterTests2b")
    def MountTopotekNetwork(self):
        '''test Topotek gimbal connected via a network port rather than a serial port'''
        self.set_parameters({
            "MNT1_TYPE": 12,      # Topotek
            "CAM1_TYPE": 4,       # Mount
            "NET_ENABLE": 1,
            "NET_P1_TYPE": 3,     # TCP client
            "NET_P1_PROTOCOL": 8,  # gimbal
            "NET_P1_IP0": 127,
            "NET_P1_IP1": 0,
            "NET_P1_IP2": 0,
            "NET_P1_IP3": 1,
            "NET_P1_PORT": 15005,
        })
        # the simulated gimbal listens on a TCP socket rather than
        # being attached to one of the autopilot's serial ports:
        self.customise_SITL_commandline(["--net-device=topotek:15005"])
        # the gimbal sends its replies out of the interface named in the
        # address field of our requests, so this only works if the
        # driver tells the gimbal we are talking to it over the network
        self.mount_check_camera_information(
            "Topotek", "SIM_TP",
            expected_fw_version=1,
            expected_cap_flags=0x6C3,
        )
        # command an angle and check the gimbal reports reaching it,
        # which requires traffic in both directions:
        self.set_mount_mode(mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p1=-30,  # pitch angle in degrees
            p7=mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
        )
        self.wait_mount_roll_pitch_yaw_deg(p=-30)

    @testcase(bucket="CopterTests1d")
    def MountViewPro(self):
        '''test Viewpro gimbal using SIM_Viewpro simulator'''
        self.set_parameters({
            "MNT1_TYPE": 11,      # Viewpro
            "CAM1_TYPE": 4,       # Mount
            "SERIAL5_PROTOCOL": 8,  # gimbal
            "RC6_OPTION": 213,    # MOUNT1_PITCH
        })
        self.customise_SITL_commandline(["--serial5=sim:viewpro:"])
        # firmware "Sys20220101" from SIM_Viewpro: major=20, minor=22, patch=1
        # firmware_version = major | (minor<<8) | (patch<<16) = 0x011614
        # cap flags: CAPTURE_VIDEO | CAPTURE_IMAGE | HAS_BASIC_ZOOM | HAS_BASIC_FOCUS |
        #            HAS_TRACKING_POINT | HAS_TRACKING_RECTANGLE
        self.mount_check_camera_information(
            "Viewpro", "SIM_VP",
            expected_fw_version=0x011614,
            expected_cap_flags=0x6C3,
        )
        # Viewpro roll uses 12-bit encoding (0..4095 = -90..+90 deg); the
        # midpoint 2047.5 is not an integer so neutral 0 deg cannot be
        # represented exactly.  Max quantisation error is 180/(4095*2) ≈ 0.022 deg.
        # pitch_rc_neutral=1818: with RC6 min=1000 max=2000 trim=1500 and
        # default MNT1_PITCH_MIN=-90 / MNT1_PITCH_MAX=20, norm_input=0.636
        # maps to exactly 0 deg pitch.
        # constrain_sysid_target=False: the driver does not yet clamp pitch to
        # MNT1_PITCH_MAX before sending, so the 68-deg sysid test is skipped
        # until AP_Mount_Viewpro::send_target_angles enforces the limit.
        self.mount_test_body(pitch_rc_neutral=1818, neutral_tol_deg=0.05,
                             do_rate_tests=False, constrain_sysid_target=False)
