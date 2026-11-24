'''
Copter autotest: OpticalFlowCalibration

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class OpticalFlowCalibration(TestGroup):
    @testcase(bucket="CopterTests1a")
    def OpticalFlowCalibration(self):
        '''test optical flow calibration'''
        ex = None
        self.context_push()
        try:

            self.set_parameter("SIM_FLOW_ENABLE", 1)
            self.set_parameter("FLOW_TYPE", 10)
            self.set_analog_rangefinder_parameters()

            # RC9 starts/stops calibration
            self.set_parameter("RC9_OPTION", 158)

            # initialise flow scaling parameters to incorrect values
            self.set_parameter("FLOW_FXSCALER", -200)
            self.set_parameter("FLOW_FYSCALER", 200)

            self.reboot_sitl()

            # ensure calibration is off
            self.set_rc(9, 1000)

            # takeoff to 10m in loiter
            self.takeoff(10, mode="LOITER", require_absolute=True, timeout=720)

            # start calibration
            self.set_rc(9, 2000)

            tstart = self.get_sim_time()
            timeout = 90
            veh_dir_tstart = self.get_sim_time()
            veh_dir = 0
            while self.get_sim_time_cached() - tstart < timeout:
                # roll and pitch vehicle until samples collected
                # change direction of movement every 2 seconds
                if self.get_sim_time_cached() - veh_dir_tstart > 2:
                    veh_dir_tstart = self.get_sim_time()
                    veh_dir = veh_dir + 1
                    if veh_dir > 3:
                        veh_dir = 0
                if veh_dir == 0:
                    # move right
                    self.set_rc_from_map({1: 1800, 2: 1500})
                if veh_dir == 1:
                    # move left
                    self.set_rc_from_map({1: 1200, 2: 1500})
                if veh_dir == 2:
                    # move forward
                    self.set_rc_from_map({1: 1500, 2: 1200})
                if veh_dir == 3:
                    # move back
                    self.set_rc_from_map({1: 1500, 2: 1800})

            # return sticks to center
            self.set_rc_from_map({1: 1500, 2: 1500})

            # stop calibration (not actually necessary)
            self.set_rc(9, 1000)

            # check scaling parameters have been restored to values near zero
            flow_scalar_x = self.get_parameter("FLOW_FXSCALER")
            flow_scalar_y = self.get_parameter("FLOW_FYSCALER")
            if ((flow_scalar_x > 30) or (flow_scalar_x < -30)):
                raise NotAchievedException("FlowCal failed to set FLOW_FXSCALER correctly")
            if ((flow_scalar_y > 30) or (flow_scalar_y < -30)):
                raise NotAchievedException("FlowCal failed to set FLOW_FYSCALER correctly")

        except Exception as e:  # noqa: BLE001
            self.print_exception_caught(e)
            ex = e

        self.disarm_vehicle(force=True)
        self.context_pop()
        self.reboot_sitl()

        if ex is not None:
            raise ex
