'''
Rover autotest: RangeFinder

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RangeFinder(TestGroup):
    @testcase(bucket="Rover")
    def RangeFinder(self):
        '''Test RangeFinder'''
        # the following magic numbers correspond to the post locations in SITL
        home_string = "%s,%s,%s,%s" % (51.8752066, 14.6487840, 54.15, 231)

        rangefinder_params = {
            "SIM_SONAR_ROT": 0,
        }
        rangefinder_params.update(self.analog_rangefinder_parameters())

        self.set_parameters(rangefinder_params)
        self.customise_SITL_commandline([
            "--home", home_string,
        ])
        self.wait_ready_to_arm()
        if self.mavproxy is not None:
            self.mavproxy.send('script /tmp/post-locations.scr\n')
        self.context_push()
        self.context_set_message_rate_hz('RANGEFINDER', self.sitl_streamrate())
        m = self.assert_receive_message('RANGEFINDER', very_verbose=True)
        self.context_pop()
        if m.voltage == 0:
            raise NotAchievedException("Did not get non-zero voltage")
        want_range = 10
        if abs(m.distance - want_range) > 0.5:
            raise NotAchievedException("Expected %fm got %fm" % (want_range, m.distance))
