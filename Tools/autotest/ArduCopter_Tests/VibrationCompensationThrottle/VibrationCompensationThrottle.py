'''
Copter autotest: VibrationCompensationThrottle

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class VibrationCompensationThrottle(TestGroup):
    @testcase(bucket="CopterTests1c")
    def VibrationCompensationThrottle(self):
        '''check throttle does not oscillate while vibration compensation is active'''
        self.context_push()

        self.takeoff(20, mode="LOITER")

        # simulate accel bias caused by high vibration
        self.set_parameters({
            'SIM_ACC1_BIAS_Z': 2,
            'SIM_ACC2_BIAS_Z': 2,
            'SIM_ACC3_BIAS_Z': 2,
        })
        self.wait_statustext("Vibration compensation ON", timeout=30)

        # count large sample-to-sample throttle changes.  A stable
        # controller only sees these when the EKF resets its height
        # estimate; an over-gained one swings the throttle continuously
        self.set_message_rate_hz('VFR_HUD', 10)
        samples = []

        def collect_throttle(mav, m):
            if m.get_type() == 'VFR_HUD':
                samples.append(m.throttle)
        self.install_message_hook_context(collect_throttle)

        self.delay_sim_time(30, "collect throttle samples")

        steps = [abs(a - b) for a, b in zip(samples, samples[1:])]
        if len(steps) < 200:
            raise NotAchievedException("insufficient samples (%u throttle steps)" % len(steps))
        large_steps = len([s for s in steps if s > 15])
        self.progress("%u of %u throttle steps over 15%%" % (large_steps, len(steps)))
        if large_steps > 20:
            raise NotAchievedException("Throttle oscillating under vibration compensation (%u large steps)" % large_steps)

        self.disarm_vehicle(force=True)

        # revert simulated accel bias and reboot to restore EKF health
        self.context_pop()
        self.reboot_sitl()
