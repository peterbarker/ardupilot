'''
Rover autotest: MAV_SYSID_32bit

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_SYSID_32bit(TestGroup):
    @testcase(bucket="Rover")
    def MAV_SYSID_32bit(self):
        '''test 32 bit MAV_SYSID'''
        if not hasattr(mavutil.mavlink, "MAVLINK_IFLAG_SYSID32"):
            raise NotAchievedException("pymavlink is too old for 32 bit system IDs")

        # sysid values must round-trip losslessly through the float32
        # parameter transport, so stay below 2^24 for now
        sysid = 100000
        # The vehicle applies the ID immediately. Switch the test connection
        # before waiting for replies, including the parameter helper's timesync.
        self.send_set_parameter_direct("MAV_SYSID", sysid)
        self.mav.target_system = sysid
        with self.mavlink_target_system_context():
            try:
                self.wait_heartbeat(timeout=60)
                m = self.wait_heartbeat()
                if m.get_srcSystem() != sysid:
                    raise NotAchievedException("Did not get 32 bit sysid, got %u" % m.get_srcSystem())
                hdr = m.get_header()
                if not (hdr.incompat_flags & mavutil.mavlink.MAVLINK_IFLAG_SYSID32):
                    raise NotAchievedException("expected MAVLINK_IFLAG_SYSID32 to be set")

                # parameter fetch at the new sysid
                if int(self.get_parameter("MAV_SYSID")) != sysid:
                    raise NotAchievedException("MAV_SYSID readback failed")

                self.reboot_sitl()
                if int(self.get_parameter("MAV_SYSID")) != sysid:
                    raise NotAchievedException("Wide MAV_SYSID did not survive reboot")

                self.set_parameter("FOLL_ENABLE", 1)
                for run_cmd in (self.run_cmd, self.run_cmd_int):
                    # Preserve legacy truncation, including positive fractions below 1.
                    for target in (0.5, 1.5, 254.75, 255, 256, 100000.5, (1 << 24) - 1):
                        run_cmd(mavutil.mavlink.MAV_CMD_DO_FOLLOW, p1=target)
                        if int(self.get_parameter("FOLL_SYSID")) != int(target):
                            raise NotAchievedException("Unexpected follow target after truncation")
                    # Legacy range comparisons trap on NaN with SITL float exceptions enabled.
                    for target in (0, -1, float('inf'), 1 << 24, 0xFFFFFFFF):
                        run_cmd(mavutil.mavlink.MAV_CMD_DO_FOLLOW, p1=target,
                                want_result=mavutil.mavlink.MAV_RESULT_DENIED)
                    if int(self.get_parameter("FOLL_SYSID")) != (1 << 24) - 1:
                        raise NotAchievedException("Invalid follow command changed target")

                # our own GCS with a 32 bit source system, and a targeted
                # message each way
                mav2 = mavutil.mavlink_connection(
                    "tcp:localhost:%u" % self.adjust_ardupilot_port(5763),
                    robust_parsing=True,
                    source_system=70000,
                    source_component=7,
                )
                mav2.mav.param_request_read_send(sysid, 1, b"MAV_SYSID", -1)
                m = mav2.recv_match(type='PARAM_VALUE', blocking=True, timeout=10)
                if m is None:
                    raise NotAchievedException("no PARAM_VALUE for 32 bit source system")
                if m.param_id != "MAV_SYSID" or int(m.param_value) != sysid:
                    raise NotAchievedException("bad PARAM_VALUE %s" % str(m))
                mav2.close()

                # targeted mission-protocol round trip (the mission upload
                # helpers hard-code target system 1, so do this by hand)
                self.mav.mav.mission_request_list_send(sysid, 1, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
                m = self.assert_receive_message('MISSION_COUNT', timeout=10)
                if m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
                    raise NotAchievedException("bad MISSION_COUNT")

                self.wait_ready_to_arm()
                self.arm_vehicle()
                self.disarm_vehicle()
            finally:
                # restore the old sysid
                self.send_set_parameter_direct("MAV_SYSID", 1)
                self.mav.target_system = 1
                self.wait_heartbeat(timeout=60)
                self.wait_heartbeat()
