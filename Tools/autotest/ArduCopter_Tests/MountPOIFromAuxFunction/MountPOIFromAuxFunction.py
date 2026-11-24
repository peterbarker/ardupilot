'''
Copter autotest: MountPOIFromAuxFunction

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import WaitAndMaintainAttitude

from test_discovery import TestGroup
from test_discovery import testcase


class MountPOIFromAuxFunction(TestGroup):
    @testcase(bucket="CopterTests2a")
    def MountPOIFromAuxFunction(self):
        '''test we can lock onto a lat/lng/alt with the flick of a switch'''
        self.install_terrain_handlers_context()
        self.MountPOIFromAuxFunction_Main(enable=1)
        self.MountPOIFromAuxFunction_Main(enable=0)

    def MountPOIFromAuxFunction_Main(self, enable):
        self.setup_servo_mount()
        self.progress(f">>>>>>>  Testing with TERRAIN_ENABLE = {enable} >>>>>>>>>>>>")
        self.set_parameters({
                "RC10_OPTION": 186,
                "MNT1_RC_RATE": 0,
                "TERRAIN_ENABLE": enable,
                "SIM_TERRAIN": enable,
                })
        self.reboot_sitl()  # to handle MNT1_TYPE changing # to handle MNT1_TYPE changing
        self.wait_ready_to_arm()

        self.takeoff(10, mode='GUIDED')
        self.set_rc(6, 1000) # tilt mount down 45 degs
        self.delay_sim_time(.1, reason="mount to move")  # allow mount to move

        self.progress("checking mount angles")
        mount_roll, mount_pitch, mount_yaw, mount_yaw_is_absolute = self.get_mount_roll_pitch_yaw_deg()
        assert -46 <= mount_pitch <= -44, f"Initial Mount Pitch is out of range: {mount_pitch}"

        self.set_rc(10, 1900)  # engage poi lock
        self.delay_sim_time(5, reason="POI computation")  # allow time to compute POI
        self.set_rc(10, 1500)  # revert mode, keep POI

        self.fly_guided_move_local(100, 100, 70)   # move to new position
        WaitAndMaintainAttitude(self, 0, 0, epsilon=1, minimum_duration=1, timeout=5).run()

        self.set_rc(10, 1900)  # re-engage poi lock and check angles again
        self.delay_sim_time(.1, reason="mount to move")  # allow mount to move

        mount_roll, mount_pitch, mount_yaw, mount_yaw_is_absolute = self.get_mount_roll_pitch_yaw_deg()
        assert (-178 <= mount_yaw <= -176), f"Mount Yaw2 is out of range: {mount_yaw}"
        assert -26 <= mount_pitch <= -24, f"Mount Pitch2 is out of range: {mount_pitch}"
        self.progress(f"Mount Pitch2 = {mount_pitch}. Mount Yaw2 = {mount_yaw}")

        self.set_rc(10, 1500)  # return to RC target mode and check that mount reverts to initial angles
        self.delay_sim_time(.1, reason="mount to move")  # allow mount to move

        mount_roll, mount_pitch, mount_yaw, mount_yaw_is_absolute = self.get_mount_roll_pitch_yaw_deg()
        assert -46 <= mount_pitch <= -44, f"Mount Pitch3 is out of range: {mount_pitch}"
        assert -1 <= mount_yaw <= 1, f"Mount Yaw3 is out of range: {mount_yaw}"
        self.progress(f"Mount Pitch3 = {mount_pitch}. Mount Yaw3 = {mount_yaw}")

        self.change_mode('RTL')
        self.wait_disarmed()
        self.assert_at_home()
