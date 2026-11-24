'''
Copter autotest: AHRSAutoTrim

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AHRSAutoTrim(TestGroup):
    @testcase(bucket="CopterTests2d")
    def AHRSAutoTrim(self):
        '''calibrate AHRS trim using RC input'''
        self.progress("Making earth frame same as body frame")  # because I'm lazy
        self.takeoff(5, mode='GUIDED')
        self.guided_achieve_heading(0, direction=1, accuracy=1)
        self.do_land()

        self.set_parameters({
            'RC9_OPTION': 182,
        })

        param_last_check_time = 0
        for mode in ['STABILIZE', 'ALT_HOLD']:
            self.set_parameters({
                'AHRS_TRIM_X': 0.1,
                'AHRS_TRIM_Y': -0.1,
            })
            self.takeoff(mode=mode)
            self.progress("Ensuring the bad trims actually cause a drift")
            self.wait_groundspeed(1.5, 1000, timeout=30)
            self.set_rc(9, 2000)
            tstart = self.get_sim_time()
            while True:
                now = self.get_sim_time_cached()
                if now - tstart > 30:
                    raise ValueError(f"Failed to reduce trims in {mode}!")
                lpn = self.assert_receive_message('LOCAL_POSITION_NED')
                delta = 40
                deadband = 0.1  # m/s; don't trim against residual drift
                roll_input = 1500
                if lpn.vx > deadband:
                    roll_input -= delta
                elif lpn.vx < -deadband:
                    roll_input += delta

                pitch_input = 1500
                if lpn.vy > deadband:
                    pitch_input += delta
                elif lpn.vy < -deadband:
                    pitch_input -= delta
                self.set_rc_from_map({
                    1: roll_input,
                    2: pitch_input,
                }, quiet=True)

                # check parameters once/second:
                if now - param_last_check_time > 1:
                    param_last_check_time = now
                    trim_x = self.get_parameter('AHRS_TRIM_X', verbose=False)
                    trim_y = self.get_parameter('AHRS_TRIM_Y', verbose=False)
                    self.progress(f"trim_x={trim_x} trim_y={trim_y}")
                    if abs(trim_x) < 0.005 and abs(trim_y) < 0.005:
                        self.progress("Good AHRS trims")
                        self.set_rc_from_map({
                            1: 1500,
                            2: 1500,
                        }, quiet=True)
                        self.wait_groundspeed(0, 0.3, minimum_duration=5, timeout=30)
                        break
            self.context_collect('STATUSTEXT')
            self.set_rc(9, 1000)
            self.wait_statustext('Trim saved', check_context=True)
            self.context_stop_collecting('STATUSTEXT')
            self.do_land()
            self.set_rc_default()

        self.progress("Landing should cancel the autotrim")
        self.takeoff(10, mode='STABILIZE')
        self.context_collect('STATUSTEXT')
        self.set_rc(9, 2000)
        self.wait_statustext('AutoTrim running', check_context=True)
        self.do_land()
        self.wait_statustext('AutoTrim cancelled', check_context=True)
        self.set_rc(9, 1000)

        self.progress("Changing mode to LOITER")
        self.takeoff(10, mode='STABILIZE')
        self.context_collect('STATUSTEXT')
        self.set_rc(9, 2000)
        self.wait_statustext('AutoTrim running', check_context=True)
        self.change_mode('LOITER')
        self.wait_statustext('AutoTrim cancelled', check_context=True)
        self.do_land()
        self.set_rc(9, 1000)

        # we are not at the home location - reboot so the next test starts there
        self.reboot_sitl()
