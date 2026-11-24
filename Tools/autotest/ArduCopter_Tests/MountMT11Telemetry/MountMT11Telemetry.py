'''
Copter autotest: MountMT11Telemetry

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra
from pymavlink import mavutil
from pymavlink import quaternion

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import WaitAndMaintainAttitude

from test_discovery import TestGroup
from test_discovery import testcase


class MountMT11Telemetry(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MountMT11Telemetry(self):
        '''verify MT11 targeting uses only telemetry requested on its private link'''
        self.set_parameters({
            "MNT1_TYPE": 6,
            "CAM1_TYPE": 6,
            "SERIAL1_PROTOCOL": 2,
            "SERIAL2_PROTOCOL": -1,  # SERIAL0/1/5 map to MAV1/2/3
            "SERIAL5_PROTOCOL": 2,
            "MAV3_OPTIONS": 2,
            "MAV3_POSITION": 0,
            "MNT1_ATT_RATE": 0,
        })
        # Bridge the private serial connection to the simulated gimbal so we
        # can inspect requests and withhold/alter the actual telemetry input.
        self.customise_SITL_commandline([
            "--serial5=tcp:5775",
            "--net-device=mt11:5776",
        ])
        saved_mavfile_global = mavutil.mavfile_global
        vehicle = mavutil.mavlink_connection("tcp:127.0.0.1:5775")
        gimbal = None
        hook = None
        try:
            gimbal = mavutil.mavlink_connection("tcp:127.0.0.1:5776")
            gimbal.target_system = self.sysid_thismav()
            mavutil.mavfile_global = saved_mavfile_global
            blocked = set()
            requests = {}
            altitude_offset_mm = 0
            position_type = 'GLOBAL_POSITION_INT'
            attitude_type = 'AUTOPILOT_STATE_FOR_GIMBAL_DEVICE'
            explicit_yaw_frame = False
            check_horizon = False
            max_lean = 0
            bank_attitude = None
            last_bank_command_ms = 0

            def bridge(mav, message):
                nonlocal max_lean, last_bank_command_ms
                if bank_attitude is not None and message.get_type() == 'SYSTEM_TIME':
                    if message.time_boot_ms - last_bank_command_ms >= 100:
                        self.mav.mav.set_attitude_target_send(
                            0, self.sysid_thismav(), 1, 7, bank_attitude, 0, 0, 0, 0.5)
                        last_bank_command_ms = message.time_boot_ms
                if check_horizon and message.get_type() == 'ATTITUDE':
                    max_lean = max(max_lean, abs(math.degrees(message.roll)), abs(math.degrees(message.pitch)))
                for source, destination in ((gimbal, vehicle), (vehicle, gimbal)):
                    for _ in range(100):
                        packet = source.recv_match()
                        if packet is None:
                            break
                        if source is gimbal:
                            if check_horizon and packet.get_type() == 'GIMBAL_DEVICE_ATTITUDE_STATUS':
                                roll, pitch, _ = quaternion.Quaternion(packet.q).euler
                                if abs(math.degrees(roll)) > 5 or abs(math.degrees(pitch) + 15) > 5:
                                    raise NotAchievedException(
                                        "MT11 did not maintain horizon-referenced roll/pitch: %.2f %.2f" %
                                        (math.degrees(roll), math.degrees(pitch)))
                            if (packet.get_type() == 'COMMAND_LONG' and
                                    packet.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL):
                                message_id = int(packet.param1)
                                if packet.param2 != 100000:
                                    raise NotAchievedException("Unexpected MT11 telemetry interval")
                                requests[message_id] = requests.get(message_id, 0) + 1
                        else:
                            if packet.get_type() in (position_type, attitude_type):
                                if packet.get_msgId() not in requests:
                                    raise NotAchievedException("Unrequested MT11 telemetry: %s" % packet.get_type())
                            if packet.get_type() in ('ATTITUDE', 'SYSTEM_TIME', 'VFR_HUD'):
                                raise NotAchievedException("Normal stream reached MT11 private link")
                            if packet.get_type() in blocked:
                                continue
                            if explicit_yaw_frame and packet.get_type() == 'GIMBAL_DEVICE_SET_ATTITUDE':
                                earth = packet.flags & mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_LOCK
                                packet.flags &= ~mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_LOCK
                                packet.flags |= (mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME if earth else
                                                 mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME)
                                destination.mav.srcSystem = packet.get_srcSystem()
                                destination.mav.srcComponent = packet.get_srcComponent()
                                destination.mav.send(packet)
                                continue
                            if packet.get_type() == position_type and altitude_offset_mm:
                                packet.alt += altitude_offset_mm
                                destination.mav.srcSystem = packet.get_srcSystem()
                                destination.mav.srcComponent = packet.get_srcComponent()
                                destination.mav.send(packet)
                                continue
                        destination.write(packet.get_msgbuf())

            hook = bridge
            self.install_message_hook(hook)
            self.wait_camera_initialised(1)
            self.wait_ready_to_arm()

            def wait_gimbal_health(healthy, timeout=10):
                tstart = self.get_sim_time()
                while self.get_sim_time_cached() - tstart < timeout:
                    self.mav.recv_match(blocking=True, timeout=0.1)
                    status = gimbal.messages.get('GIMBAL_DEVICE_ATTITUDE_STATUS')
                    if status is not None and (status.failure_flags == 0) == healthy:
                        return
                raise NotAchievedException("MT11 did not become healthy=%s" % healthy)

            def wait_fresh_gimbal_status(timeout=5):
                tstart = self.get_sim_time()
                start_ms = round(tstart * 1000)
                while self.get_sim_time_cached() - tstart < timeout:
                    self.mav.recv_match(blocking=True, timeout=0.1)
                    status = gimbal.messages.get('GIMBAL_DEVICE_ATTITUDE_STATUS')
                    # SIM_MT11 shares the vehicle clock. Require a sample
                    # generated after this wait, not cached or queued feedback.
                    if status is not None and status.time_boot_ms > start_ms:
                        if status.failure_flags != 0:
                            raise NotAchievedException("MT11 reported unhealthy attitude status")
                        return status
                raise NotAchievedException("MT11 did not provide fresh attitude status")

            self.progress("Checking MT11 requested both private-link telemetry streams")
            wait_gimbal_health(True)
            for message_id in (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                               mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE):
                if message_id not in requests:
                    raise NotAchievedException("MT11 did not request message %u" % message_id)

            here = self.get_location(frame=AltFrame.ABOVE_HOME)
            target = self.offset_location_ne(here, 100, 100)
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                x=int(target.lat * 1e7),
                y=int(target.lng * 1e7),
                z=here.get_alt_m(AltFrame.ABOVE_HOME) + 50,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            )
            expected_pitch = math.degrees(math.atan2(50, math.sqrt(2) * 100))
            self.wait_mount_roll_pitch_yaw_deg(p=expected_pitch, y=45)

            self.progress("Checking MT11 uses received MSL altitude for ROI")
            altitude_offset_mm = 100000
            self.wait_mount_roll_pitch_yaw_deg(p=-expected_pitch, y=45)
            altitude_offset_mm = 0
            self.wait_mount_roll_pitch_yaw_deg(p=expected_pitch, y=45)

            for message_type in (position_type, attitude_type):
                self.progress("Checking MT11 loss and recovery of %s" % message_type)
                before = dict(requests)
                blocked.add(message_type)
                wait_gimbal_health(False)
                self.delay_sim_time(2, reason="allow MT11 to re-request missing telemetry")
                message_id = getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_' + message_type)
                if requests.get(message_id, 0) <= before.get(message_id, 0):
                    raise NotAchievedException("MT11 did not re-request %s" % message_type)
                blocked.clear()
                wait_gimbal_health(True)
                self.wait_mount_roll_pitch_yaw_deg(p=expected_pitch, y=45)

            self.progress("Checking MT11 ROI tracking after aircraft movement")
            self.takeoff(20, mode="GUIDED")
            self.fly_guided_move_local(50, 0, 20)
            self.wait_mount_roll_pitch_yaw_deg(
                p=math.degrees(math.atan2(30, math.hypot(50, 100))), tolerance=1, minimum_duration=1)

            self.progress("Checking MT11 heading-relative commands while the aircraft banks")
            explicit_yaw_frame = True
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                         p1=-15, p2=0, p3=float('nan'), p4=float('nan'))
            self.wait_mount_roll_pitch_yaw_deg(r=0, p=-15, y=0, tolerance=1, minimum_duration=1)
            # Check frame conventions at a sustained tilt, not against a hard
            # instantaneous bound during vehicle/EKF attitude transients.
            vehicle_yaw = self.assert_receive_message('ATTITUDE').yaw
            bank_attitude = mavextra.euler_to_quat([math.radians(20), math.radians(10), vehicle_yaw])
            self.change_mode('GUIDED_NOGPS')
            WaitAndMaintainAttitude(self, 20, 10, epsilon=1, minimum_duration=1, timeout=20).run()
            self.wait_mount_roll_pitch_yaw_deg(r=0, p=-15, y=0, tolerance=1, minimum_duration=1)
            check_horizon = True
            self.delay_sim_time(2, reason="verify MT11 horizon lock while holding the aircraft tilted")
            wait_fresh_gimbal_status()
            if max_lean < 10:
                raise NotAchievedException("Aircraft did not lean enough to test MT11 horizon lock")

            check_horizon = False
            bank_attitude = None
            self.change_mode('GUIDED')
            # Zero-rate control holds the attitude at the switch, so finish
            # braking and let angle control settle before changing modes.
            WaitAndMaintainAttitude(self, 0, 0, epsilon=1, minimum_duration=1, timeout=20).run()
            self.wait_mount_roll_pitch_yaw_deg(r=0, p=-15, y=0, tolerance=1, minimum_duration=1)
            check_horizon = True
            self.progress("Checking MT11 earth-frame zero rate while the aircraft yaws")
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                         p1=float('nan'), p2=float('nan'), p3=0, p4=0,
                         p5=mavutil.mavlink.GIMBAL_MANAGER_FLAGS_YAW_LOCK)
            self.delay_sim_time(1, reason="allow explicit earth-frame rate target to reach MT11")
            status = wait_fresh_gimbal_status()
            if not status.flags & mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME:
                raise NotAchievedException("MT11 ignored explicit earth-frame command")
            start_yaw = quaternion.Quaternion(status.q).euler[2]
            self.run_cmd(mavutil.mavlink.MAV_CMD_CONDITION_YAW, p1=180, p2=30, p3=1, p4=0)
            self.wait_heading(180)
            self.delay_sim_time(1, reason="settle aircraft yaw after MT11 earth-frame hold")
            status = wait_fresh_gimbal_status()
            if not status.flags & mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME:
                raise NotAchievedException("MT11 lost earth-frame yaw hold")
            end_yaw = quaternion.Quaternion(status.q).euler[2]
            yaw_error = math.degrees(end_yaw - start_yaw)
            if abs((yaw_error + 180) % 360 - 180) > 5:
                raise NotAchievedException("MT11 earth-frame zero rate followed aircraft yaw")
            check_horizon = False
            self.land_and_disarm()
        finally:
            if hook is not None:
                self.remove_message_hook(hook)
            if gimbal is not None:
                gimbal.close()
            vehicle.close()
            mavutil.mavfile_global = saved_mavfile_global
