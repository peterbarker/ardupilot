'''
Copter autotest: GyroFFTContinuousAveraging

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GyroFFTContinuousAveraging(TestGroup):
    @testcase(bucket="CopterTests2b", attempts=4, speedup=8)
    def GyroFFTContinuousAveraging(self):
        """Use dynamic harmonic notch with FFT averaging to control motor noise
           with harmonic matching of the first harmonic."""
        self.test_gyro_fft_harmonic(True)
