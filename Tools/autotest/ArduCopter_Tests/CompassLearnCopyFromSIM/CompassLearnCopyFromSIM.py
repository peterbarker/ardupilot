'''
Copter autotest: CompassLearnCopyFromSIM

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import EKF_MAG_OFFSETS_SAVED

from test_discovery import TestGroup
from test_discovery import testcase


class CompassLearnCopyFromSIM(TestGroup):
    @testcase(bucket="CopterTests2a")
    def CompassLearnCopyFromSIM(self):
        '''test COMPASS_LEARN=2 saves the ideal offsets from the SIM AHRS backend'''
        # the SIM AHRS backend reports the offsets the simulation is
        # applying, so it behaves like a perfectly-converged estimator.
        # That means no flying is required and we can assert exactly.
        sim_offsets = {
            "SIM_MAG1_OFS_X": 120, "SIM_MAG1_OFS_Y": -80, "SIM_MAG1_OFS_Z": 60,
            "SIM_MAG2_OFS_X": -70, "SIM_MAG2_OFS_Y": 110, "SIM_MAG2_OFS_Z": -50,
            "SIM_MAG3_OFS_X": 90, "SIM_MAG3_OFS_Y": 40, "SIM_MAG3_OFS_Z": -100,
        }
        # what each compass should end up with once the offsets are copied:
        expected_offsets = {
            "COMPASS_OFS_X": 120, "COMPASS_OFS_Y": -80, "COMPASS_OFS_Z": 60,
            "COMPASS_OFS2_X": -70, "COMPASS_OFS2_Y": 110, "COMPASS_OFS2_Z": -50,
            "COMPASS_OFS3_X": 90, "COMPASS_OFS3_Y": 40, "COMPASS_OFS3_Z": -100,
        }
        # deliberately wrong, but only mildly so; the compasses must stay
        # consistent enough that the vehicle will still pass prearm:
        offset_error = 25
        wrong_offsets = {x: expected_offsets[x] + offset_error for x in expected_offsets}

        self.set_parameters(sim_offsets)
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,  # use the SIM AHRS backend
        })
        self.reboot_sitl()

        # deliberately wrong offsets, so we can see them being corrected:
        self.set_parameters(wrong_offsets)

        self.start_subtest("offsets are left alone when COMPASS_LEARN is off")
        self.set_parameter("COMPASS_LEARN", 0)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.disarm_vehicle()
        self.assert_parameter_values(wrong_offsets)
        self.assert_EV_count(EKF_MAG_OFFSETS_SAVED, 0)

        self.start_subtest("offsets are saved on disarm when COMPASS_LEARN=2")
        self.set_parameter("COMPASS_LEARN", 2)  # 2 is Copy-from-EKF
        self.arm_vehicle()
        self.disarm_vehicle()
        # all three compasses should have been saved; the SIM backend
        # returns offsets for every instance, unlike a single EKF core:
        self.assert_parameter_values(expected_offsets)
        self.assert_EV_count(EKF_MAG_OFFSETS_SAVED, 1)
        self.reboot_sitl()
        self.assert_parameter_values(expected_offsets)
