'''
Copter autotest: Landing

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class Landing(TestGroup):
    @testcase(bucket="CopterTests2a")
    def Landing(self):
        """Test landing the aircraft."""

        def check_landing_speeds(land_speed_high, land_speed_low, land_alt_low, land_speed_high_accuracy=0.1):
            self.progress("Checking landing speeds (speed_high=%f speed_low=%f alt_low=%f" %
                          (land_speed_high, land_speed_low, land_alt_low))
            land_high_maintain = 5
            land_low_maintain = land_alt_low / land_speed_low / 2

            takeoff_alt = (land_high_maintain * land_speed_high + land_alt_low) + 20
            # this is pretty rough, but takes *so much longer* in LOITER
            self.takeoff(takeoff_alt, mode='STABILIZE', timeout=200, takeoff_throttle=2000)
            # check default landing speeds:
            self.change_mode('LAND')
            # ensure higher-alt descent rate:
            self.wait_descent_rate(land_speed_high,
                                   minimum_duration=land_high_maintain,
                                   accuracy=land_speed_high_accuracy)
            self.wait_descent_rate(land_speed_low)
            # ensure we transition to low descent rate at correct height:
            self.assert_altitude(land_alt_low, relative=True)
            # now make sure we maintain that descent rate:
            self.wait_descent_rate(land_speed_low, minimum_duration=land_low_maintain)
            self.wait_disarmed()

        # test the defaults.  By default LAND_SPD_HIGH_MS is 0 so
        # WP_SPD_DN is used
        check_landing_speeds(
            self.get_parameter("WP_SPD_DN"),
            self.get_parameter("LAND_SPD_MS"),
            self.get_parameter("LAND_ALT_LOW_M")
        )

        def test_landing_speeds(land_speed_high, land_speed_low, land_alt_low, **kwargs):
            self.set_parameters({
                "LAND_SPD_HIGH_MS": land_speed_high,
                "LAND_SPD_MS": land_speed_low,
                "LAND_ALT_LOW_M": land_alt_low
            })
            check_landing_speeds(land_speed_high, land_speed_low, land_alt_low, **kwargs)

        test_landing_speeds(
            5,  # descent speed high
            1,  # descent speed low
            30,  # transition altitude
            land_speed_high_accuracy=0.5
        )
