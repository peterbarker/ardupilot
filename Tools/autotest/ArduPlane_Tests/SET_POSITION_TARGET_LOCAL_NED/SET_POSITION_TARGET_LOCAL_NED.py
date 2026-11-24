'''
Plane autotest: SET_POSITION_TARGET_LOCAL_NED

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import MAV_POS_TARGET_TYPE_MASK
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SET_POSITION_TARGET_LOCAL_NED(TestGroup):
    def send_set_position_target_local_ned_z(self, frame, z):
        self.mav.mav.set_position_target_local_ned_send(
            0, # time_boot_ms
            self.sysid_thismav(),
            1, # target component
            frame,
            MAV_POS_TARGET_TYPE_MASK.ALT_ONLY,
            0, # x
            0, # y
            z,
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yaw_rate
        )

    @testcase(bucket="PlaneTests1b")
    def SET_POSITION_TARGET_LOCAL_NED(self):
        '''test changing altitude in GUIDED with SET_POSITION_TARGET_LOCAL_NED'''
        takeoff_alt = 50
        self.takeoff(takeoff_alt, mode='TAKEOFF')
        self.context_collect('STATUSTEXT')

        def assert_no_altitude_change_text():
            self.delay_sim_time(5, reason="vehicle to process message")
            for m in self.context_collection('STATUSTEXT'):
                if m.text.startswith("Change alt to"):
                    raise NotAchievedException(f"Unexpected altitude change: {m.text}")

        self.start_subtest("ignored outside GUIDED")
        self.change_mode('LOITER')
        self.send_set_position_target_local_ned_z(mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, -30)
        assert_no_altitude_change_text()

        self.change_mode('GUIDED')

        self.start_subtest("ignored in frames other than MAV_FRAME_LOCAL_OFFSET_NED")
        self.send_set_position_target_local_ned_z(mavutil.mavlink.MAV_FRAME_LOCAL_NED, -30)
        assert_no_altitude_change_text()

        self.start_subtest("offset the altitude")
        want_alt = self.get_altitude(relative=True)
        for z in -30, 20:
            want_alt -= z
            self.context_clear_collection('STATUSTEXT')
            self.send_set_position_target_local_ned_z(mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, z)
            m = self.wait_statustext("Change alt to", check_context=True)
            target_alt = float(m.text.split()[-1])
            if abs(target_alt - want_alt) > 5:
                raise NotAchievedException(f"Want target altitude near {want_alt} got {target_alt}")
            self.wait_altitude(target_alt-3, target_alt+3, relative=True, minimum_duration=10, timeout=60)
            want_alt = target_alt

        self.fly_home_land_and_disarm()
