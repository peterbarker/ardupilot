'''
Plane autotest: Volz

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class Volz(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def Volz(self):
        '''test Volz serially-connected'''
        volz_motor_mask = ((1 << 0) | (1 << 1) | (1 << 3) | (1 << 8) | (1 << 9) | (1 << 11))
        self.set_parameters({
            'SERIAL5_PROTOCOL': 14,
            'SERVO_VOLZ_MASK': volz_motor_mask,
            'RTL_AUTOLAND': 2,

            'SIM_VOLZ_ENA': 1,
            'SIM_VOLZ_MASK': volz_motor_mask,
        })
        # defaults file not working?
        self.set_parameters({
            "SERVO2_REVERSED":  0,  # elevator

            "SERVO9_FUNCTION": 4,

            "SERVO10_FUNCTION": 19,  # elevator

            "SERVO12_FUNCTION": 21,  # rudder
            "SERVO12_REVERSED":  1,  # rudder

        })
        self.customise_SITL_commandline([
            "--serial5=sim:volz",
        ], model="plane-redundant",
                                        )
        self.wait_ready_to_arm()
        self.takeoff()
        self.change_mode('FBWA')
        straight_and_level_text = "straight-and-level"
        self.send_statustext(straight_and_level_text)
        self.delay_sim_time(2, reason="straight-and-level flight data to log")
        self.progress("sticking servo with constant deflection")
        self.set_rc(1, 1400)
        self.change_mode('MANUAL')
        self.delay_sim_time(0.5, reason="servo deflection to establish")
        self.progress("Failing servo")
        self.set_parameter('SIM_VOLZ_FMASK', 1)
        self.set_rc(1, 1500)
        self.change_mode('FBWA')
        aileron_failed_text = "aileron has been failed"
        self.send_statustext(aileron_failed_text)
        self.delay_sim_time(15, reason="aileron failure data collection")
        self.set_parameter('SIM_VOLZ_FMASK', 0)

        log_filepath = self.current_onboard_log_filepath()
        # terminate vehicle in-flight so our tests aren't fooled by the
        # "flying home" data:
        self.reboot_sitl(force=True)

        self.progress("Inspecting DFReader to ensure servo failure is recorded in the log")
        dfreader = self.dfreader_for_path(log_filepath)
        while True:
            m = dfreader.recv_match(type=['MSG'])
            if m is None:
                raise NotAchievedException("Did not see straight_and_level_text")
            if m.Message == "SRC=250/250:" + straight_and_level_text:
                break

        self.progress("Ensuring deflections are close to zero in straight-and-level flight")
        chan1_good = False
        chan9_good = False
        while not (chan1_good and chan9_good):
            m = dfreader.recv_match()
            if m is None:
                raise NotAchievedException("Did not see chan1 and chan9 as close-to-0")
            if m.get_type() != 'CSRV':
                continue
            if m.Id == 0 and abs(m.Pos) < 3:
                chan1_good = True
            elif m.Id == 8 and abs(m.Pos) < 3:
                chan9_good = True

        while True:
            m = dfreader.recv_match(type=['MSG'])
            if m is None:
                raise NotAchievedException("Did not see aileron_failed_text")
            if m.Message == "SRC=250/250:" + aileron_failed_text:
                break

        self.progress("Checking servo9 is deflected")
        while True:
            # m = dfreader.recv_match(type=['CSRV'])
            m = dfreader.recv_match()
            if m is None:
                raise NotAchievedException("Did not see chan9 deflection")
            if m.get_type() != 'CSRV':
                continue
            if m.Id != 8:
                continue
            if m.Pos < 20:
                continue
            self.progress(f"Chan9 is deflected ({m})")
            break

        self.progress("Ensuring the vehicle stabilised with a single aileron")
        attitude_good_count = 0
        while attitude_good_count < 5:
            m = dfreader.recv_match()
            if m is None:
                raise NotAchievedException("Did not see good attitude")
            if m.get_type() != 'ATT':
                continue
            if abs(m.Roll) >= 5:
                attitude_good_count = 0
                continue
            attitude_good_count += 1
        self.progress(f"Attitude is stabilised ({m})")

        self.progress("Ensure the roll integrator is wound up")
        while True:
            m = dfreader.recv_match()
            if m is None:
                raise NotAchievedException("Did not see wound-up roll integrator")
            if m.get_type() != 'PIDR':
                continue
            if m.I > 5:
                self.progress(f"Roll integrator is wound up ({m})")
                break

        self.progress("Checking that aileron is stuck at some deflection")
        good_count = 0
        while good_count < 5:
            m = dfreader.recv_match()
            if m is None:
                raise NotAchievedException("Did not see csrv Pos/PosCmd discrepancy")
            if m.get_type() != 'CSRV':
                continue
            if m.Id != 0:
                continue
            delta = abs(m.Pos - m.PosCmd)
            if delta <= 20:
                self.progress(f"CSRV Pos/PosCmd {delta=:.2f} BAD {m}")
                good_count = 0
                continue
            self.progress(f"CSRV Pos/PosCmd {delta=:.2f} OK {m}")
            good_count += 1
