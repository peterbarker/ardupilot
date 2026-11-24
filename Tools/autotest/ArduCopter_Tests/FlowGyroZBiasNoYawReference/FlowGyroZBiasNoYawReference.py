'''
Copter autotest: FlowGyroZBiasNoYawReference

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FlowGyroZBiasNoYawReference(TestGroup):
    @testcase(bucket="CopterTests2a")
    def FlowGyroZBiasNoYawReference(self):
        '''Z gyro bias is not learned from optical flow without a fused yaw reference'''
        # With optical flow as the velocity source and nothing observing
        # heading, a flow sensor error can be absorbed as a Z gyro bias. The
        # simulated gyro has no bias, so any Z bias learned here is phantom.
        # Each case removes the yaw reference a different way, at arming:
        # yaw fusion during the climb shrinks the bias variance enough that
        # the flow gain into the bias is too small to show anything in a
        # short flight.
        cases = [
            ("no yaw source", None, {"EK3_SRC1_YAW": 0, "SIM_GPS1_ENABLE": 0}, None),
            ("compass lost", None, {"EK3_SRC1_YAW": 1, "SIM_GPS1_ENABLE": 0},
             {"SIM_MAG1_FAIL": 1, "SIM_MAG2_FAIL": 1, "SIM_MAG3_FAIL": 1}),
            ("GPS yaw lost", "copter-gps-for-yaw.parm", {"EK3_SRC1_YAW": 2},
             {"SIM_GPS1_ENABLE": 0, "SIM_GPS2_ENABLE": 0}),
        ]
        max_bias_dps = 0.1
        failures = []
        for (name, params_file, params, yaw_loss) in cases:
            self.start_subtest(name)
            self.context_push()
            if params_file is not None:
                self.load_default_params_file(params_file)
            self.set_parameters({
                "SIM_FLOW_ENABLE": 1,
                "FLOW_TYPE": 10,
                "FLOW_FXSCALER": 400,  # a flow scale error for the bias to absorb
                "SIM_TERRAIN": 0,
            })
            self.set_parameters(params)
            self.configure_EKFs_to_use_optical_flow_instead_of_GPS()
            self.set_analog_rangefinder_parameters()
            self.reboot_sitl()
            if params_file is not None:
                self.wait_gps_fix_type_gte(6, message_type="GPS2_RAW", verbose=True)

            self.change_mode('LOITER')
            self.wait_ready_to_arm(require_absolute=False)
            self.zero_throttle()
            self.arm_vehicle()
            if yaw_loss is not None:
                self.set_parameters(yaw_loss)
            self.takeoff(10, mode='LOITER', require_absolute=False, takeoff_throttle=1800)
            # a box with turns, so the flow error is seen on changing headings
            for _ in range(4):
                self.set_rc(2, 1300)
                self.delay_sim_time(5, "flying a leg")
                self.set_rc(2, 1500)
                self.set_rc(4, 1700)
                self.delay_sim_time(2, "turning")
                self.set_rc(4, 1500)
                self.delay_sim_time(3, "stopping")
            # LAND leans hard against the flow scale error on touchdown and
            # never detects the landing; ALT_HOLD has no position controller
            self.change_mode('ALT_HOLD')
            self.set_rc(3, 1000)
            self.wait_disarmed(timeout=120)
            self.set_rc(3, 1500)

            dfreader = self.dfreader_for_current_onboard_log()
            bias_dps = {}
            armed = False
            flying = False
            flow_fusion_started = False
            aiding_stopped = 0
            while True:
                m = dfreader.recv_match(type=['ARM', 'EV', 'MSG', 'XKF1'])
                if m is None:
                    break
                mtype = m.get_type()
                if mtype == 'ARM':
                    armed = m.ArmState == 1
                elif mtype == 'EV':
                    # only the flight is under test: a flow focus floor withholds
                    # the flow on the ground, and aiding may stop after touchdown
                    if m.Id == 28:  # LogEvent::NOT_LANDED
                        flying = True
                    elif m.Id == 18:  # LogEvent::LAND_COMPLETE
                        flying = False
                elif mtype == 'XKF1':
                    bias_dps[m.C] = max(bias_dps.get(m.C, 0), abs(m.GZ))
                elif "fusing optical flow" in m.Message:
                    flow_fusion_started = True
                elif armed and flying and "stopped aiding" in m.Message:
                    aiding_stopped += 1
            self.progress(f"{name}: max |GZ| {bias_dps} flow fusion started {flow_fusion_started} "
                          f"aiding stopped {aiding_stopped}")
            # without flow fusion a zero bias proves nothing. With flow as the
            # only aiding source, aiding stops once no flow update has passed
            # its innovation gate for 5 s
            if not flow_fusion_started or aiding_stopped > 0:
                failures.append(f"{name}: optical flow was not fused throughout the flight")
            worst = max(bias_dps.values(), default=0)
            if worst > max_bias_dps:
                failures.append(f"{name}: learned a {worst:.2f} deg/s Z gyro bias")
            self.context_pop()
            self.reboot_sitl()
        if len(failures):
            raise NotAchievedException("; ".join(failures))
