'''
Copter autotest: ThrowMode

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ThrowMode(TestGroup):
    @testcase(bucket="CopterTests2a")
    def ThrowMode(self):
        '''Fly Throw Mode'''
        # test boomerang mode:
        self.progress("Throwing vehicle away")
        self.set_parameters({
            "THROW_NEXTMODE": 6,
            "SIM_SHOVE_Z": -30,
            "SIM_SHOVE_X": -20,
        })
        self.change_mode('THROW')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        try:
            self.set_parameter("SIM_SHOVE_TIME", 500)
        except ValueError:
            # the shove resets this to zero
            pass

        tstart = self.get_sim_time()
        self.wait_mode('RTL')
        max_good_tdelta = 15
        tdelta = self.get_sim_time() - tstart
        self.progress("Vehicle in RTL")
        self.wait_rtl_complete()
        self.progress("Vehicle disarmed")
        if tdelta > max_good_tdelta:
            raise NotAchievedException("Took too long to enter RTL: %fs > %fs" %
                                       (tdelta, max_good_tdelta))
        self.progress("Vehicle returned")
