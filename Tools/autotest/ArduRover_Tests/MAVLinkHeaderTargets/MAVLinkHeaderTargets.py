'''
Rover autotest: MAVLinkHeaderTargets

AP_FLAKE8_CLEAN
'''

import time

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAVLinkHeaderTargets(TestGroup):
    @testcase(bucket="Rover")
    def MAVLinkHeaderTargets(self):
        """Wide target headers route independently of source ID width."""
        sysid = 100000
        self.send_set_parameter_direct("MAV_SYSID", sysid)
        self.mav.target_system = sysid
        with self.mavlink_target_system_context():
            mav2 = None

            def service_primary_link():
                # Waiting on mav2 does not run the primary link's hooks.
                self.drain_all_pexpects()
                self.drain_mav()

            try:
                mav2 = mavutil.mavlink_connection(
                    "tcp:localhost:%u" % self.adjust_ardupilot_port(5763),
                    source_system=42, source_component=7)
                self.wait_heartbeat(timeout=60)
                self.assert_receive_message('HEARTBEAT', mav=mav2, timeout=30,
                                            delay_fn=service_primary_link)
                # Distinct parameters prevent delayed retry replies from satisfying
                # the next source-ID case; PARAM_VALUE has no destination field.
                for source, name in ((42, "MAV_SYSID"), (70000, "MAV_GCS_SYSID"), (0xFFFFFFFF, "MAV_GCS_SYSID_HI")):
                    mav2.mav.srcSystem = source
                    request = mav2.mav.param_request_read_encode(sysid, 1, name.encode('ascii'), -1)
                    expected_flags = 4 | (2 if source > 255 else 0)
                    deadline = time.monotonic() + 30
                    next_send = 0
                    attempts = 0
                    reply = None
                    while time.monotonic() < deadline:
                        service_primary_link()
                        if time.monotonic() >= next_send:
                            attempts += 1
                            self.progress("Requesting %s from source %u to target %u (attempt %u)" %
                                          (name, source, sysid, attempts))
                            mav2.mav.send(request)
                            if request.get_header().incompat_flags != expected_flags:
                                raise NotAchievedException("Source and target widths are not independent")
                            next_send = time.monotonic() + 1
                        reply = mav2.recv_match(type='PARAM_VALUE', condition='PARAM_VALUE.param_id == "%s"' % name,
                                                blocking=True, timeout=0.1)
                        if reply is not None:
                            break
                    if reply is None:
                        raise NotAchievedException("No %s reply from target %u to source %u after %u attempts" %
                                                   (name, sysid, source, attempts))
                    if reply.get_srcSystem() != sysid:
                        raise NotAchievedException("Parameter reply came from the wrong vehicle")
                    if reply.get_target_system() is not None or reply.get_header().incompat_flags & 4:
                        raise NotAchievedException("Targetless reply acquired a target header")
                    # Sharing the low target byte must not alias this vehicle.
                    deadline = time.monotonic() + 1
                    next_send = 0
                    while time.monotonic() < deadline:
                        service_primary_link()
                        if time.monotonic() >= next_send:
                            mav2.mav.param_request_read_send(sysid + 256, 1, b"FRAME_CLASS", -1)
                            next_send = time.monotonic() + 0.2
                        if mav2.recv_match(type='PARAM_VALUE', condition='PARAM_VALUE.param_id == "FRAME_CLASS"',
                                           blocking=True, timeout=0.1) is not None:
                            raise NotAchievedException("Processed request for a different wide target")
            finally:
                if mav2 is not None:
                    mav2.close()
                self.send_set_parameter_direct("MAV_SYSID", 1)
                self.mav.target_system = 1
                self.wait_heartbeat(timeout=60)
