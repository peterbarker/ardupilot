'''
QuadPlane autotest: CompassLearnCopyFromEKFAffinity

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import EKF_MAG_OFFSETS_SAVED

from test_discovery import TestGroup
from test_discovery import testcase


class CompassLearnCopyFromEKFAffinity(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def CompassLearnCopyFromEKFAffinity(self):
        '''check EKF-learned offsets are saved for several compasses at once'''
        # A pure fixed-wing never satisfies the finalInflightMagInit
        # condition in NavEKF3_core::getMagOffsets(): the block that
        # requests the in-flight mag/yaw reset is guarded by
        # !assume_zero_sideslip() (AP_NavEKF3_MagFusion.cpp), which is
        # false while flying forward.  A quadplane in a VTOL mode is not
        # "fly forward", so taking off in QLOITER does get the reset done;
        # it then persists, as only onGround clears it and that stays
        # false while armed.
        #
        # With EK3 compass affinity each core is pinned to its own compass
        # (AP_NavEKF3_Measurements.cpp update_mag_selection) and the
        # frontend asks every core for each instance in turn, so a single
        # disarm can save offsets for more than one compass.
        self.set_parameters({
            "EK3_AFFINITY": 4,  # 4 is EnableCompassAffinity
            "EK3_IMU_MASK": 3,  # two IMUs, so two cores, so two compasses
        })
        self.reboot_sitl()

        self.takeoff(30, 'QLOITER')

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

        # transition to fixed wing and get some height to play with:
        self.change_mode('FBWA')
        self.set_rc(3, 2000)
        self.wait_altitude(250, 350, relative=True, timeout=300)

        # rolling and looping gives the roll and pitch diversity needed to
        # separate the body-frame biases from the earth field estimate;
        # there's a 5e-6 variance check before the offsets are good!
        for _ in range(8):
            self.progress("Starting roll")
            self.change_mode('MANUAL')
            self.set_rc(1, 1000)
            self.wait_roll(-150, accuracy=90)
            self.wait_roll(150, accuracy=90)
            self.wait_roll(0, accuracy=90)
            self.set_rc(1, 1500)
            self.change_mode('FBWA')
            self.wait_level_flight()

            self.progress("Starting loop")
            self.change_mode('MANUAL')
            self.set_rc(2, 1000)
            self.wait_pitch(-60, accuracy=20)
            self.wait_pitch(0, accuracy=20)
            self.set_rc(2, 1500)
            self.change_mode('FBWA')
            self.wait_level_flight()

            self.wait_altitude(250, 400, relative=True, timeout=300)

        self.set_rc(3, 1500)

        # land in a VTOL mode and disarm; we are high and a long way from
        # home after all of that, so this is not quick:
        self.change_mode('QRTL')
        self.wait_disarmed(timeout=600)
        # both compasses should have been learned and saved on that disarm:
        expected_offsets = {
            "COMPASS_OFS_X": new_compass_ofs_x,
            "COMPASS_OFS2_X": new_compass2_ofs_x,
        }
        self.assert_parameter_values(expected_offsets, epsilon=30)
        self.assert_EV_count(EKF_MAG_OFFSETS_SAVED, 1)
        self.reboot_sitl()
        self.assert_parameter_values(expected_offsets, epsilon=30)
