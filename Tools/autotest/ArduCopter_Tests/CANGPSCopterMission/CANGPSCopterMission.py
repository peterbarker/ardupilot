'''
Copter autotest: CANGPSCopterMission

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CANGPSCopterMission(TestGroup):
    @testcase(bucket="CAN")
    def CANGPSCopterMission(self):
        '''fly mission which tests normal operation alongside CAN GPS'''
        self.set_parameters({
            "CAN_P1_DRIVER": 1,
            "GPS1_TYPE": 9,
            "GPS2_TYPE": 9,
            # disable simulated GPS, so only via DroneCAN
            "SIM_GPS1_ENABLE": 0,
            "SIM_GPS2_ENABLE": 0,
            # this ensures we use DroneCAN baro and compass
            "SIM_BARO_COUNT" : 0,
            "SIM_MAG1_DEVID" : 0,
            "SIM_MAG2_DEVID" : 0,
            "SIM_MAG3_DEVID" : 0,
            "COMPASS_USE2"   : 0,
            "COMPASS_USE3"   : 0,
            # use DroneCAN rangefinder
            "RNGFND1_TYPE" : 24,
            "RNGFND1_MAX" : 110.00,
            # use DroneCAN battery monitoring, and enforce with a arming voltage
            "BATT_MONITOR" : 8,
            "BATT_ARM_VOLT" : 12.0,
        })

        self.context_push()
        # enable only GPS arming check during ordering test
        self.set_parameter("ARMING_SKIPCHK", ~(1 << 3))
        self.context_collect('STATUSTEXT')

        self.reboot_sitl()
        # Test UAVCAN GPS ordering working
        gps1_det_text = self.wait_text("GPS 1: specified as DroneCAN.*", regex=True, check_context=True)
        gps2_det_text = self.wait_text("GPS 2: specified as DroneCAN.*", regex=True, check_context=True)
        gps1_nodeid = int(gps1_det_text.split('-')[1])
        gps2_nodeid = int(gps2_det_text.split('-')[1])
        if gps1_nodeid is None or gps2_nodeid is None:
            raise NotAchievedException("GPS not ordered per the order of Node IDs")

        self.context_stop_collecting('STATUSTEXT')

        GPS_Order_Tests = [[gps2_nodeid, gps2_nodeid, gps2_nodeid, 0,
                            "PreArm: Same Node Id {} set for multiple GPS".format(gps2_nodeid)],
                           [gps1_nodeid, int(gps2_nodeid/2), gps1_nodeid, 0,
                            "Selected GPS Node {} not set as instance {}".format(int(gps2_nodeid/2), 2)],
                           [int(gps1_nodeid/2), gps2_nodeid, 0, gps2_nodeid,
                            "Selected GPS Node {} not set as instance {}".format(int(gps1_nodeid/2), 1)],
                           [gps1_nodeid, gps2_nodeid, gps1_nodeid, gps2_nodeid, ""],
                           [gps2_nodeid, gps1_nodeid, gps2_nodeid, gps1_nodeid, ""],
                           [gps1_nodeid, 0, gps1_nodeid, gps2_nodeid, ""],
                           [0, gps2_nodeid, gps1_nodeid, gps2_nodeid, ""]]
        for case in GPS_Order_Tests:
            self.progress("############################### Trying Case: " + str(case))
            self.set_parameters({
                "GPS1_CAN_OVRIDE": case[0],
                "GPS2_CAN_OVRIDE": case[1],
            })
            self.drain_mav()
            self.context_collect('STATUSTEXT')
            self.reboot_sitl()
            # detection statustexts for both GPSs are emitted together
            # shortly after boot.  Wait for any GPS expected to be
            # present first, so boot latency is absorbed by that wait;
            # a GPS expected to be absent then only needs a short
            # timeout, as a (wrongly) emitted detection statustext
            # would already be in the collected context.
            gps_det_texts = {}
            for instance in sorted((1, 2), key=lambda i: case[1 + i] == 0):
                timeout = 20 if case[1 + instance] != 0 else 5
                try:
                    gps_det_texts[instance] = self.wait_text(
                        "GPS %u: specified as DroneCAN.*" % instance,
                        regex=True,
                        check_context=True,
                        timeout=timeout,
                    )
                except AutoTestTimeoutException:
                    gps_det_texts[instance] = None
            gps1_det_text = gps_det_texts[1]
            gps2_det_text = gps_det_texts[2]

            self.context_stop_collecting('STATUSTEXT')
            self.change_mode('LOITER')
            if case[2] == 0 and case[3] == 0:
                if gps1_det_text or gps2_det_text:
                    raise NotAchievedException("Failed ordering for requested CASE:", case)

            if case[2] == 0 or case[3] == 0:
                if bool(gps1_det_text is not None) == bool(gps2_det_text is not None):
                    print(gps1_det_text)
                    print(gps2_det_text)
                    raise NotAchievedException("Failed ordering for requested CASE:", case)

            if gps1_det_text:
                if case[2] != int(gps1_det_text.split('-')[1]):
                    raise NotAchievedException("Failed ordering for requested CASE:", case)
            if gps2_det_text:
                if case[3] != int(gps2_det_text.split('-')[1]):
                    raise NotAchievedException("Failed ordering for requested CASE:", case)
            if len(case[4]):
                self.context_collect('STATUSTEXT')
                self.run_cmd(
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    p1=1,  # ARM
                    timeout=10,
                    want_result=mavutil.mavlink.MAV_RESULT_FAILED,
                )
                self.wait_statustext(case[4], check_context=True)
                self.context_stop_collecting('STATUSTEXT')
        self.progress("############################### All GPS Order Cases Tests Passed")
        self.progress("############################### Test Healthy Prearm check")
        self.set_parameter("ARMING_SKIPCHK", 0)
        self.stop_sup_program(instance=0)
        self.start_sup_program(instance=0, args="-M")
        self.stop_sup_program(instance=1)
        self.start_sup_program(instance=1, args="-M")
        self.context_collect('STATUSTEXT')
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            p1=1,  # ARM
            timeout=10,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )
        # the peripherals take wall-clock time to restart, so at high
        # speedup a large amount of simulation time may pass before
        # their maintenance-mode NodeStatus is seen; the recurring
        # prearm display emits the message once it is
        self.wait_statustext(".*Node .* unhealthy", check_context=True, regex=True, timeout=600)
        self.stop_sup_program(instance=0)
        self.start_sup_program(instance=0)
        self.stop_sup_program(instance=1)
        self.start_sup_program(instance=1)
        self.context_stop_collecting('STATUSTEXT')
        self.context_pop()

        # the restarted peripherals take wall-clock time to boot and
        # their GPSs must then deliver on-time fixes for long enough
        # for the GPS timing health filters to recover.  The prearm
        # SYS_STATUS bit reflects the full arming checks, including
        # health of every GPS instance; require it healthy
        # continuously before flying
        self.progress("Waiting for sustained prearm health after peripheral restart")
        tstart = self.get_sim_time()
        healthy_since = None
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 600:
                raise NotAchievedException("Peripherals did not return to sustained health")
            m = self.assert_receive_message('SYS_STATUS')
            if m.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK:
                if healthy_since is None:
                    healthy_since = now
                if now - healthy_since > 20:
                    break
            else:
                healthy_since = None

        self.set_parameters({
            # use DroneCAN ESCs for flight
            "CAN_D1_UC_ESC_BM" : 0x0f,
            # this stops us using local servo output, guaranteeing we are
            # flying on DroneCAN ESCs
            "SIM_CAN_SRV_MSK" : 0xFF,
        })

        self.CopterMission()
