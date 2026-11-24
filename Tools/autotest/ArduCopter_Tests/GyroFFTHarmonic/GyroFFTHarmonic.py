'''
Copter autotest: GyroFFTHarmonic

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GyroFFTHarmonic(TestGroup):
    @testcase(bucket="CopterTests2c", attempts=4, speedup=8)
    def GyroFFTHarmonic(self):
        """Use dynamic harmonic notch to control motor noise with harmonic matching of the first harmonic."""
        self.test_gyro_fft_harmonic(False)
