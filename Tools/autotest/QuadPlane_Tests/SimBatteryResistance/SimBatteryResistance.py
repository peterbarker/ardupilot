'''
QuadPlane autotest: SimBatteryResistance

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SimBatteryResistance(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def SimBatteryResistance(self):
        '''check SIM_BATT_RES_OHM controls simulated battery voltage sag under load'''
        self.set_parameters({
            # an analog monitor reports the simulated (sagged) battery voltage
            "BATT_MONITOR": 4,  # 4 is analog volt+curr
            # unlimited capacity pins the resting voltage at SIM_BATT_VOLTAGE, so
            # any drop in the reported voltage is purely sag (current * resistance)
            "SIM_BATT_CAP_AH": 0,
            "SIM_BATT_VOLTAGE": 12.6,
            # keep battery failsafes out of the way while we deliberately sag the pack
            "BATT_LOW_VOLT": 0,
            "BATT_CRT_VOLT": 0,
            "BATT_FS_LOW_ACT": 0,
            "BATT_FS_CRT_ACT": 0,
        })
        self.reboot_sitl()  # BATT_MONITOR change requires a reboot

        # put the motors under a steady hover load
        self.takeoff(20, mode="QHOVER")

        def hover_voltage():
            # let the 10Hz sim voltage filter settle, then average a few samples
            self.delay_sim_time(2, "let hover voltage settle")
            samples = 10
            total = 0
            for _ in range(samples):
                m = self.assert_receive_message('BATTERY_STATUS', timeout=5)
                total += m.voltages[0] * 0.001  # mV -> V
            return total / samples

        # with sag disabled the pack should read close to its resting voltage
        self.set_parameter("SIM_BATT_RES_OHM", 0)
        v_no_sag = hover_voltage()
        self.progress(f"hover voltage, sag disabled: {v_no_sag:.4f}V")
        if abs(v_no_sag - 12.6) > 0.2:
            raise NotAchievedException(
                f"Expected ~resting voltage with sag disabled, got {v_no_sag:.4f}V")

        # introducing resistance should sag the voltage under the same load
        hover_voltage_ohms = 0.05
        self.set_parameter("SIM_BATT_RES_OHM", hover_voltage_ohms)
        v_sag = hover_voltage()
        self.progress(f"hover voltage, {hover_voltage_ohms:.4f}ohm: {v_sag:.4f}V")
        if v_sag > v_no_sag - 0.3:
            raise NotAchievedException(
                f"Expected voltage sag with resistance, got {v_sag:.4f}V (no-sag {v_no_sag:.4f}V)")

        # more resistance should sag the voltage further still
        hover_voltage_more_ohms = 0.1
        self.set_parameter("SIM_BATT_RES_OHM", hover_voltage_more_ohms)
        v_more_sag = hover_voltage()
        self.progress(f"hover voltage, {hover_voltage_more_ohms:.4f}ohm: {v_more_sag:.4f}V")
        if v_more_sag > v_sag - 0.2:
            raise NotAchievedException(
                f"Expected more sag at higher resistance, got {v_more_sag:.4f}V"
                f" ({hover_voltage_more_ohms:.4f}ohm {v_sag:.4f}V)")

        # restore full thrust before landing so the RTL is not handicapped
        self.set_parameter("SIM_BATT_RES_OHM", 0)
        self.do_RTL()
