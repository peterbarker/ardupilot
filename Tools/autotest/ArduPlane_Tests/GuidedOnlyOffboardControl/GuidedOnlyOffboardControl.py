'''
Plane autotest: GuidedOnlyOffboardControl

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import MAV_POS_TARGET_TYPE_MASK
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GuidedOnlyOffboardControl(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def GuidedOnlyOffboardControl(self):
        '''check offboard altitude targets are ignored outside GUIDED, and attitude targets in AVOID_ADSB'''
        self.set_parameters({
            "ADSB_TYPE": 1,
            "AVD_ENABLE": 1,
            "AVD_F_ACTION": mavutil.mavlink.MAV_COLLISION_ACTION_MOVE_HORIZONTALLY,
        })
        self.reboot_sitl()
        self.takeoff(50, mode='TAKEOFF')

        def send_altitude_target(frame, alt):
            self.mav.mav.set_position_target_global_int_send(
                0, # time_boot_ms
                self.sysid_thismav(),
                1, # target component
                frame,
                MAV_POS_TARGET_TYPE_MASK.ALT_ONLY,
                0, # lat
                0, # lon
                alt,
                0, # vx
                0, # vy
                0, # vz
                0, # afx
                0, # afy
                0, # afz
                0, # yaw
                0, # yaw_rate
            )

        def send_zero_thrust_target():
            # use thrust only:
            type_mask = (mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE |
                         mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE |
                         mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE |
                         mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE)
            self.mav.mav.set_attitude_target_send(
                0, # time_boot_ms
                self.sysid_thismav(),
                1, # target component
                type_mask,
                [1, 0, 0, 0],
                0, # roll rate
                0, # pitch rate
                0, # yaw rate
                0, # thrust
            )

        # LOITER flies at the altitude of the navigation target which
        # SET_POSITION_TARGET_GLOBAL_INT changes in GUIDED:
        self.start_subtest("SET_POSITION_TARGET_GLOBAL_INT ignored in LOITER")
        self.change_mode('LOITER')
        self.delay_sim_time(10, reason="vehicle to settle into LOITER")
        start_alt = self.get_altitude(relative=True)
        send_altitude_target(mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, start_alt+40)
        self.wait_altitude(start_alt-5, start_alt+5, relative=True, minimum_duration=15, timeout=30)

        self.start_subtest("SET_POSITION_TARGET_GLOBAL_INT with an invalid frame")
        self.change_mode('GUIDED')
        self.context_collect('STATUSTEXT')
        send_altitude_target(mavutil.mavlink.MAV_FRAME_LOCAL_NED, start_alt+40)
        # the text is long enough to be sent in chunks, so match on the first:
        self.wait_statustext("Invalid coord frame in SET_POSITION_TARGET_GLOBAL", check_context=True)
        self.wait_altitude(start_alt-5, start_alt+5, relative=True, minimum_duration=15, timeout=30)

        # AVOID_ADSB runs the GUIDED controllers, which would act on an
        # accepted attitude or thrust target.  (Avoidance resets the
        # navigation target continually, so an accepted altitude
        # target would not be seen here.)
        self.start_subtest("SET_ATTITUDE_TARGET ignored in AVOID_ADSB")
        self.change_mode('LOITER')

        def send_threat():
            # keep a threat just beside the vehicle so it stays in AVOID_ADSB:
            self.test_adsb_send_threatening_adsb_message(self.get_location(), offset_ne=(0, 30))

        send_threat()
        self.wait_mode('AVOID_ADSB')
        start_alt = self.get_altitude(relative=True)
        throttles = []
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 15:
            send_threat()
            send_zero_thrust_target()
            self.assert_mode_is('AVOID_ADSB')
            throttle = self.assert_receive_message('VFR_HUD').throttle
            if self.get_sim_time_cached() - tstart > 1:
                # an accepted target would have taken effect by now
                throttles.append(throttle)
            alt = self.get_altitude(relative=True)
            if abs(alt - start_alt) > 10:
                raise NotAchievedException(f"Altitude changed from {start_alt} to {alt} in AVOID_ADSB")
            self.delay_sim_time(0.2, reason="rate-limit targets")
        low = [t for t in throttles if t < 10]
        self.progress(f"AVOID_ADSB throttle samples: {throttles}")
        if len(throttles) < 10 or len(low) > 0.2 * len(throttles):
            raise NotAchievedException(
                f"Throttle below 10% in {len(low)} of {len(throttles)} samples; zero thrust target followed?")
        self.wait_for_collision_threat_to_clear()

        self.fly_home_land_and_disarm()
