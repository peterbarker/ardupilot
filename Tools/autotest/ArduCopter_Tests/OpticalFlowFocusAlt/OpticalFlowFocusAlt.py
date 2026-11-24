'''
Copter autotest: OpticalFlowFocusAlt

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class OpticalFlowFocusAlt(TestGroup):
    @testcase(bucket="CopterTests1e")
    def OpticalFlowFocusAlt(self):
        '''Below FLOW_ALT_MIN the EKF does not fuse optical flow so bad flow cannot drive a phantom velocity'''
        # The check is driven by the altitude above terrain (HAGL) the EKF uses for flow fusion.  FLOW_ALT_MIN
        # is set far above any real sensor so the vehicle can hover below it long enough to measure.
        self.set_parameters({
            "SIM_FLOW_ENABLE": 1,
            "FLOW_TYPE": 10,
            "SIM_GPS1_ENABLE": 0,
        })
        self.configure_EKFs_to_use_optical_flow_instead_of_GPS()
        self.set_analog_rangefinder_parameters()

        hover_alt_m = 2.0

        def fly_with_bad_flow(flow_alt_min):
            self.set_parameters({"FLOW_ALT_MIN": flow_alt_min, "SIM_FLOW_OFS_X": 0})
            self.reboot_sitl()
            self.wait_ready_to_arm(require_absolute=False, timeout=120)
            # flow is not healthy while stationary on the ground, so climb in ALT_HOLD
            # before entering a mode that needs a position estimate
            self.takeoff(
                altitude_min=5,
                mode='ALT_HOLD',
                require_absolute=False,
                takeoff_throttle=1700,
            )
            self.change_mode('GUIDED')
            self.send_position_target_local_ned(0, 0, hover_alt_m)
            self.wait_altitude(
                hover_alt_m - 0.3,
                hover_alt_m + 0.3,
                relative=True,
                minimum_duration=3,
                timeout=90,
            )
            # measure in ALT_HOLD, which leaves a phantom velocity in the estimate.  A
            # position-controlled mode flies it away instead, hiding the effect.
            self.hover()
            self.change_mode('ALT_HOLD')
            # only messages from the hover below FLOW_ALT_MIN are of interest
            self.context_collect('STATUSTEXT')
            self.context_clear_collection('STATUSTEXT')
            # a flow rate offset reads as motion that is not happening, as an unfocused
            # sensor does near the ground
            self.set_parameter("SIM_FLOW_OFS_X", 1.0)

        self.start_subtest("Below FLOW_ALT_MIN the flow is not fused")
        fly_with_bad_flow(3.0)
        self.wait_groundspeed(0, 0.5, minimum_duration=15, timeout=25)
        # the EKF loses flow aiding after 5s without flow to fuse but must not start it again
        if self.statustext_in_collections("started relative aiding"):
            raise NotAchievedException("Relative aiding restarted below FLOW_ALT_MIN")
        self.set_parameter("SIM_FLOW_OFS_X", 0)
        self.disarm_vehicle(force=True)

        self.start_subtest("FLOW_ALT_MIN disabled: bad flow drives a phantom velocity")
        fly_with_bad_flow(0)
        self.wait_groundspeed(0.8, 1000, timeout=30)
        self.set_parameter("SIM_FLOW_OFS_X", 0)
        self.disarm_vehicle(force=True)

        self.start_subtest("Above FLOW_ALT_MIN the flow is still fused")
        fly_with_bad_flow(1.0)
        self.wait_groundspeed(0.8, 1000, timeout=30)
        self.set_parameter("SIM_FLOW_OFS_X", 0)
        self.disarm_vehicle(force=True)
