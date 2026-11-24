'''
Copter autotest: EK3_RNG_USE_HGT

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class EK3_RNG_USE_HGT(TestGroup):
    @testcase(bucket="CopterTests1a")
    def EK3_RNG_USE_HGT(self):
        '''basic tests for using rangefinder when speed and height below thresholds'''
        # this takes advantage of some code in send_status_report
        # which only reports terrain variance when using switch-height
        # and using the rangefinder
        self.context_push()

        self.set_analog_rangefinder_parameters()
        # set use-height to 20m (the parameter is a percentage of max range)
        self.set_parameters({
            'EK3_RNG_USE_HGT': (20 / self.get_parameter('RNGFND1_MAX')) * 100,
        })
        self.reboot_sitl()

        # add a listener that verifies rangefinder innovations look good
        global alt
        alt = None

        def verify_innov(mav, m):
            global alt
            if m.get_type() == 'GLOBAL_POSITION_INT':
                alt = m.relative_alt * 0.001  # mm -> m
                return
            if m.get_type() != 'EKF_STATUS_REPORT':
                return
            if alt is None:
                return
            if alt > 1 and alt < 8:  # 8 is very low, but it takes a long time to start to use the rangefinder again
                zero_variance_wanted = False
            elif alt > 20:
                zero_variance_wanted = True
            else:
                return
            variance = m.terrain_alt_variance
            if zero_variance_wanted and variance > 0.00001:
                raise NotAchievedException("Wanted zero variance at height %f, got %f" % (alt, variance))
            elif not zero_variance_wanted and variance == 0:
                raise NotAchievedException("Wanted non-zero variance at alt=%f, got zero" % alt)

        self.install_message_hook_context(verify_innov)

        self.takeoff(50, mode='GUIDED')
        current_alt = self.get_location().get_alt_m(AltFrame.ABSOLUTE)
        target_position = Location(
            -35.362938,
            149.165185,
            current_alt,
            AltFrame.ABSOLUTE,
        )

        self.fly_guided_move_to(target_position, timeout=300)

        self.change_mode('LAND')
        self.wait_disarmed()

        self.context_pop()

        self.reboot_sitl()
