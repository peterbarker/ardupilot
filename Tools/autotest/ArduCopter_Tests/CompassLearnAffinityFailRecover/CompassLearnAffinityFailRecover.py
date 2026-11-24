'''
Copter autotest: CompassLearnAffinityFailRecover

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CompassLearnAffinityFailRecover(TestGroup):
    @testcase(bucket="CopterTests2d")
    def CompassLearnAffinityFailRecover(self):
        '''check EKF-learned offsets are saved to the right compass after a compass fails and recovers'''
        # with EK3 compass affinity the core pinned to compass 1 moves to
        # compass 2 when compass 1 fails and returns to compass 1 when it
        # recovers.  The offsets saved at disarm must still belong to the
        # compass they were learned for.
        self.set_parameters({
            "EK3_AFFINITY": 4,  # 4 is EnableCompassAffinity
            "EK3_IMU_MASK": 3,  # two IMUs, so two cores, so two compasses
        })
        self.reboot_sitl()

        self.wait_ready_to_arm()
        self.takeoff(30, mode='ALT_HOLD')
        self.set_parameter('COMPASS_USE3', 0)
        # the firmware is about to learn and save these, so set them to
        # the values they already have; that way the suite knows what to
        # restore them to at context pop time.  set_and_save_offsets()
        # writes all three axes, not just the one we assert on:
        self.set_parameters(self.get_parameters([
            "COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z",
            "COMPASS_OFS2_X", "COMPASS_OFS2_Y", "COMPASS_OFS2_Z",
            "COMPASS_OFS3_X", "COMPASS_OFS3_Y", "COMPASS_OFS3_Z",
        ]))
        original_compass_ofs_x = self.get_parameter("COMPASS_OFS_X")
        new_compass_ofs_x = 200
        new_compass2_ofs_x = -150
        self.set_parameters({
            "SIM_MAG1_OFS_X": new_compass_ofs_x,
            "SIM_MAG2_OFS_X": new_compass2_ofs_x,
        })
        self.set_parameter("COMPASS_LEARN", 2)  # 2 is Copy-from-EKF

        self.fly_compass_learning_manoeuvres()

        # fail compass 1; the core using it switches to compass 2 and
        # has to learn compass 2's field from scratch
        self.set_parameter("SIM_MAG1_FAIL", 1)
        self.wait_statustext("EKF3 IMU0 switching to compass 1", timeout=60)
        self.fly_compass_learning_manoeuvres()

        # compass 1 recovers and affinity returns the core to it
        self.context_collect('STATUSTEXT')
        self.set_parameter("SIM_MAG1_FAIL", 0)
        self.wait_statustext("EKF3 IMU0 switching to compass 0", check_context=True, timeout=60)
        self.delay_sim_time(15, reason="compass 1 to be used again")

        self.do_RTL()
        # compass 2's offsets were learned and must be saved to compass 2
        self.assert_parameter_value("COMPASS_OFS2_X", new_compass2_ofs_x, epsilon=30)
        # compass 1 has had little time to relearn since it recovered, so
        # its offsets are either unchanged or correct, but must never be
        # compass 2's offsets
        ofs_x = self.get_parameter("COMPASS_OFS_X")
        if abs(ofs_x - original_compass_ofs_x) > 30 and abs(ofs_x - new_compass_ofs_x) > 30:
            raise NotAchievedException(
                "COMPASS_OFS_X=%f is neither the original (%f) nor the learned (%f) offset" %
                (ofs_x, original_compass_ofs_x, new_compass_ofs_x))
