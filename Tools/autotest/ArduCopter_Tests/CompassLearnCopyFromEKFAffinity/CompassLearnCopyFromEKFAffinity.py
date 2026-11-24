'''
Copter autotest: CompassLearnCopyFromEKFAffinity

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import EKF_MAG_OFFSETS_SAVED

from test_discovery import TestGroup
from test_discovery import testcase


class CompassLearnCopyFromEKFAffinity(TestGroup):
    @testcase(bucket="CopterTests2d")
    def CompassLearnCopyFromEKFAffinity(self):
        '''check EKF-learned offsets are saved for several compasses at once'''
        # with EK3 compass affinity each core is pinned to its own compass
        # (AP_NavEKF3_Measurements.cpp update_mag_selection), and the
        # frontend asks every core for each instance in turn, so a single
        # disarm can save offsets for more than one compass.  Note we
        # deliberately leave COMPASS_USE2 on, unlike CompassLearnCopyFromEKF
        # -- affinity is what stops the EKF switching away from a bad one.
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
        new_compass_ofs_x = 200
        new_compass2_ofs_x = -150
        self.set_parameters({
            "SIM_MAG1_OFS_X": new_compass_ofs_x,
            "SIM_MAG2_OFS_X": new_compass2_ofs_x,
        })
        self.set_parameter("COMPASS_LEARN", 2)  # 2 is Copy-from-EKF

        self.fly_compass_learning_manoeuvres()

        self.do_RTL()
        # both compasses should have been learned and saved on that disarm:
        expected_offsets = {
            "COMPASS_OFS_X": new_compass_ofs_x,
            "COMPASS_OFS2_X": new_compass2_ofs_x,
        }
        self.assert_parameter_values(expected_offsets, epsilon=30)
        self.assert_EV_count(EKF_MAG_OFFSETS_SAVED, 1)
        self.reboot_sitl()
        self.assert_parameter_values(expected_offsets, epsilon=30)
