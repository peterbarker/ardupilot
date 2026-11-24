'''
QuadPlane autotest: CircuitStatusScript

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CircuitStatusScript(TestGroup):
    # anomalous: ~9 minutes on CI against ~20s on a desktop; to be investigated
    @testcase(bucket="QuadPlaneTests1b")
    def CircuitStatusScript(self):
        '''test CircuitStatus lua driver against a CAN periph'''
        self.context_collect('STATUSTEXT')

        self.install_driver_script_context("CircuitStatus.lua")

        # CAN_P1_DRIVER=1 is needed so plane publishes the SITL multicast
        # sim state the periph blocks on at boot. BATT_MONITOR=8
        # (DroneCAN) gives a reference monitor fed by BatteryInfo from
        # the same periph battery the CircuitStatus messages come from.
        self.set_parameters({
            "SCR_ENABLE": 1,
            "CAN_P1_DRIVER": 1,
            "BATT_MONITOR": 8,  # DroneCAN
            "BATT2_MONITOR": 29,  # scripting
            "BATT3_MONITOR": 29,  # scripting
        })
        self.restart_SITL_frame('quadplane-can', customisations=[])

        # first boot of the script creates DCS_NUM_CIRCUITS; setting it
        # and rebooting creates the per-circuit parameters
        self.set_parameters({
            "DCS_NUM_CIRCUITS": 2,
        })
        self.reboot_sitl()
        self.wait_statustext("CircuitStatus: loaded 2 circuits", check_context=True, timeout=60)

        # per-circuit parameters are polled at runtime, so no further
        # reboot is needed. The SITL periph sends a circuit per battery
        # backend with circuit_id of instance+1; map its first battery
        # to both scripting monitors
        self.set_parameters({
            "DCS1_CIRCUIT_ID": 1,
            "DCS1_BATT_IDX": 2,
            "DCS2_CIRCUIT_ID": 1,
            "DCS2_BATT_IDX": 3,
        })

        # the scripting monitors should match the DroneCAN reference
        # monitor, with tolerance for float16 quantisation and sampling
        # time differences. SYS_STATUS gives the pack voltage of the
        # reference monitor; BATTERY_STATUS voltages[] of the DroneCAN
        # monitor holds per-cell voltages so is not comparable.
        #
        # The reference values move while we watch - the simulated
        # battery's current drifted from 8.5A to 10.75A during one
        # loaded run - so comparing against a single SYS_STATUS
        # snapshot fails whenever the wait takes any real time: track
        # the reference live and pass when a scripting reading agrees
        # with a recent reference reading.
        self.set_message_rate_hz('BATTERY_STATUS', 10)
        for instance in 1, 2:
            for field, ref_field, epsilon in [
                    ("voltages[0]", "voltage_battery", 100),
                    ("current_battery", "current_battery", 25),
            ]:
                tstart = self.get_sim_time()
                ref = None
                last_pair = None
                while True:
                    if self.get_sim_time_cached() - tstart > 60:
                        raise NotAchievedException(
                            "BATTERY_STATUS.%s (instance %u) never matched "
                            "SYS_STATUS.%s (last %s)" %
                            (field, instance, ref_field, last_pair))
                    m = self.assert_receive_message(['SYS_STATUS', 'BATTERY_STATUS'])
                    if m.get_type() == 'SYS_STATUS':
                        ref = getattr(m, ref_field)
                        continue
                    if m.id != instance or ref is None:
                        continue
                    value = getattr(m, field.rstrip(']').split('[')[0])
                    if '[' in field:
                        value = value[int(field.split('[')[1].rstrip(']'))]
                    last_pair = (value, ref)
                    if abs(value - ref) <= epsilon:
                        self.progress("%s instance %u matches reference (%s ~ %s)" %
                                      (field, instance, value, ref))
                        break
