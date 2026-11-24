'''
Copter autotest: PositionTargetGlobalInt

AP_FLAKE8_CLEAN
'''

import copy

from pymavlink import mavextra
from pymavlink import mavutil

from vehicle_test_suite import MAV_POS_TARGET_TYPE_MASK
from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PositionTargetGlobalInt(TestGroup):
    @testcase(bucket="CopterTests2c")
    def PositionTargetGlobalInt(self):
        '''check POSITION_TARGET_GLOBAL_INT reports the commanded setpoint'''
        # yaw and yaw rate are never reported, and the top nibble is always set
        PVA_MASK = (MAV_POS_TARGET_TYPE_MASK.LAST_BYTE |
                    MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE |
                    MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE)
        POS_MASK = MAV_POS_TARGET_TYPE_MASK.LAST_BYTE | MAV_POS_TARGET_TYPE_MASK.POS_ONLY

        def receive_target():
            self.drain_mav()
            return self.assert_receive_message('POSITION_TARGET_GLOBAL_INT', timeout=2, verbose=True)

        def assert_mask(m, want):
            if m.type_mask != want:
                raise NotAchievedException("type_mask=0x%04X want 0x%04X" % (m.type_mask, want))

        def assert_vector(name, got, want, tolerance):
            for (g, w) in zip(got, want):
                if abs(g - w) > tolerance:
                    raise NotAchievedException("%s=%s want %s (+/-%f)" % (name, str(got), str(want), tolerance))

        def assert_position(m, lat, lng, alt_amsl_m, tolerance):
            dist = mavextra.distance_lat_lon(m.lat_int * 1e-7, m.lon_int * 1e-7, lat, lng)
            if dist > tolerance or abs(m.alt - alt_amsl_m) > tolerance:
                raise NotAchievedException("target %fm from expected, alt=%f want %f" % (dist, m.alt, alt_amsl_m))

        def assert_not_sent():
            self.drain_mav()
            self.assert_not_receiving_message('POSITION_TARGET_GLOBAL_INT', timeout=1)

        def send_local_ned(type_mask, pos=(0, 0, 0), vel=(0, 0, 0), accel=(0, 0, 0)):
            self.mav.mav.set_position_target_local_ned_send(
                0, # timestamp
                1, # target system_id
                1, # target component id
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE,
                pos[0], pos[1], pos[2],
                vel[0], vel[1], vel[2],
                accel[0], accel[1], accel[2],
                0, # yaw
                0, # yawrate
            )

        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT, 10)
        self.takeoff(20, mode='LOITER')

        self.start_subtest("LOITER reports no target")
        assert_not_sent()

        self.change_mode('GUIDED')
        origin = self.poll_message('GPS_GLOBAL_ORIGIN')
        origin_lat = origin.latitude * 1e-7
        origin_lng = origin.longitude * 1e-7
        origin_alt_m = origin.altitude * 0.001

        def local_to_global(north, east):
            return mavextra.gps_offset(origin_lat, origin_lng, east, north)

        self.start_subtest("Guided Pos reports the position only")
        pos = (20, 10, -20)
        send_local_ned(MAV_POS_TARGET_TYPE_MASK.POS_ONLY, pos=pos)
        self.delay_sim_time(1, reason="Pos submode to take the target")
        m = receive_target()
        assert_mask(m, POS_MASK)
        (lat, lng) = local_to_global(pos[0], pos[1])
        assert_position(m, lat, lng, origin_alt_m - pos[2], 0.5)
        self.wait_distance_to_local_position(local_position=pos, distance_min=0, distance_max=1, timeout=60)

        self.start_subtest("Guided VelAccel and Accel report no target")
        send_local_ned(MAV_POS_TARGET_TYPE_MASK.POS_IGNORE |
                       MAV_POS_TARGET_TYPE_MASK.ACC_IGNORE |
                       MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE |
                       MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE, vel=(1, 0, 0))
        assert_not_sent()
        send_local_ned(MAV_POS_TARGET_TYPE_MASK.POS_IGNORE |
                       MAV_POS_TARGET_TYPE_MASK.VEL_IGNORE |
                       MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE |
                       MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE, accel=(0, 0, 0))
        assert_not_sent()

        # stream a target accelerating north; the vehicle advances the
        # position and velocity between updates using the acceleration
        pva_type_mask = MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE | MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE
        vel0 = (2, 0, 0)
        accel = (0.2, 0, 0)

        def stream_pva(duration, check):
            start = self.assert_receive_message('LOCAL_POSITION_NED')
            p0 = (start.x, start.y, pos[2])
            tstart = self.get_sim_time()
            last_check = tstart
            while True:
                now = self.get_sim_time_cached()
                t = now - tstart
                if t > duration:
                    return
                p = (p0[0] + vel0[0] * t + 0.5 * accel[0] * t * t,
                     p0[1] + vel0[1] * t + 0.5 * accel[1] * t * t,
                     p0[2])
                vel = (vel0[0] + accel[0] * t, vel0[1] + accel[1] * t, 0)
                send_local_ned(pva_type_mask, pos=p, vel=vel, accel=accel)
                m = self.assert_receive_message('POSITION_TARGET_GLOBAL_INT', timeout=2)
                if check and now - last_check > 1:
                    last_check = now
                    self.progress("PosVelAccel target: %s" % str(m))
                    assert_mask(m, PVA_MASK)
                    assert_vector("vel", (m.vx, m.vy, m.vz), vel, 0.1)
                    assert_vector("accel", (m.afx, m.afy, m.afz), accel, 0.01)
                    (lat, lng) = local_to_global(p[0], p[1])
                    assert_position(m, lat, lng, origin_alt_m - p[2], 1.0)

        self.start_subtest("Guided PosVelAccel reports position, velocity and acceleration")
        stream_pva(10, check=True)

        self.start_subtest("paused Guided PosVelAccel reports only the resume position")
        self.send_pause_command()
        self.wait_groundspeed(0, 0.5, minimum_duration=2, timeout=30)
        first = receive_target()
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 3:
            m = receive_target()
            assert_mask(m, POS_MASK)
            assert_vector("vel", (m.vx, m.vy, m.vz), (0, 0, 0), 0)
            assert_vector("accel", (m.afx, m.afy, m.afz), (0, 0, 0), 0)
            assert_position(m, first.lat_int * 1e-7, first.lon_int * 1e-7, first.alt, 0.1)

        self.start_subtest("resumed Guided PosVelAccel reports velocity again")
        self.send_resume_command()
        stream_pva(5, check=True)

        self.start_subtest("AUTO reports the destination only")
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 100, 50, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 50, 100, 20),
        ])
        self.change_mode('AUTO')
        self.wait_current_waypoint(1)
        m = receive_target()
        assert_mask(m, POS_MASK)
        dest = self.offset_location_ne(self.home_position_as_location(), 100, 50)
        dist = mavextra.distance_lat_lon(m.lat_int * 1e-7, m.lon_int * 1e-7, dest.lat, dest.lng)
        if dist > 1:
            raise NotAchievedException("AUTO target %fm from waypoint" % dist)

        self.start_subtest("FOLLOW reports the lead's offset position and velocity")
        self.change_mode('LOITER')
        self.set_parameters({
            "FOLL_ENABLE": 1,
            "FOLL_SYSID": self.mav.source_system,
            "FOLL_OFS_X": -10,
            "FOLL_OFS_TYPE": 1, # relative to lead heading
        })
        self.context_push()
        self.context_set_speedup(1)
        self.change_mode('FOLLOW')
        lead_vel_n = 3
        lead = self.get_location()
        self.location_offset_ne(lead, 20, 0)
        last_sent = 0
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 5:
                break
            if now - last_sent > 0.2:
                pos = copy.copy(lead)
                self.location_offset_ne(pos, lead_vel_n * (now - tstart), 0)
                gpi = self.mav.mav.global_position_int_encode(
                    int(now * 1000), # time_boot_ms
                    int(pos.lat * 1e7),
                    int(pos.lng * 1e7),
                    int(pos.get_alt_m(AltFrame.ABSOLUTE) * 1000), # alt in mm
                    int(pos.get_alt_m(AltFrame.ABSOLUTE) * 1000 - origin.altitude), # relative alt
                    vx=lead_vel_n * 100,
                    vy=0,
                    vz=0,
                    hdg=0
                )
                gpi.pack(self.mav.mav)
                self.mav.mav.send(gpi)
                last_sent = now
            m = self.assert_receive_message('POSITION_TARGET_GLOBAL_INT', timeout=2)
            if now - tstart > 2:
                self.progress("FOLLOW target: %s" % str(m))
                assert_mask(m, PVA_MASK)
                assert_vector("vel", (m.vx, m.vy, m.vz), (lead_vel_n, 0, 0), 0.3)
                # the lead heads north, so FOLL_OFS_X puts the target behind it
                want = copy.copy(pos)
                self.location_offset_ne(want, -10, 0)
                assert_position(m, want.lat, want.lng, want.get_alt_m(AltFrame.ABSOLUTE), 1.5)
        self.context_pop()

        self.do_RTL()
