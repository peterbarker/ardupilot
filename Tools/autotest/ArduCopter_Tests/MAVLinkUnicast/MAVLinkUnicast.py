'''
Copter autotest: MAVLinkUnicast

AP_FLAKE8_CLEAN
'''

import time

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAVLinkUnicast(TestGroup):
    @testcase(bucket="CopterTests1c")
    def MAVLinkUnicast(self):
        '''unicast links forward addressed traffic but isolate broadcasts'''
        self.set_parameters({
            "SERIAL1_PROTOCOL": 2,
            "SERIAL2_PROTOCOL": 2,
            "SERIAL5_PROTOCOL": 2,
            "SERIAL6_PROTOCOL": 2,
        })
        self.reboot_sitl()
        # MAVLink instances are one-based and assigned in serial-port order.
        self.set_parameters({"MAV2_OPTIONS": 0, "MAV3_OPTIONS": 16, "MAV4_OPTIONS": 16, "MAV5_OPTIONS": 2})
        # Copter's stream defaults are zero. Seed non-zero rates to exercise
        # the defaults used by other vehicles and previously configured links.
        for instance in (2, 3, 4):
            self.set_parameters({"MAV%u_EXTRA1" % instance: 2,
                                 "MAV%u_EXTRA3" % instance: 2,
                                 "MAV%u_POSITION" % instance: 2})
        self.reboot_sitl()
        links = {"gcs": self.mav}
        saved_mavfile_global = mavutil.mavfile_global
        try:
            for name, serial, sysid, compid in (("normal", 1, 253, 191),
                                                ("device", 2, 42, 100),
                                                ("second", 5, 43, 101),
                                                ("private", 6, 44, 102)):
                links[name] = mavutil.mavlink_connection(
                    self.sitl_serial_endpoint(serial), source_system=sysid, source_component=compid,
                    robust_parsing=True)
            mavutil.mavfile_global = saved_mavfile_global

            def collect(duration=1):
                received = {name: [] for name in links}
                tstart = self.get_sim_time_cached()
                wall_start = time.time()
                while self.get_sim_time_cached() - tstart < duration:
                    if time.time() - wall_start > 10:
                        raise AutoTestTimeoutException("Simulation stopped during routing check")
                    for name, link in links.items():
                        while (msg := link.recv_match()) is not None:
                            received[name].append(msg)
                    # Keep the simulation clock current without consuming a
                    # possible routed packet outside the collection above.
                    self.mav.select(0.01)
                return received

            for link in links.values():
                link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            collect()

            def vehicle_messages(messages, message_type=None):
                return [msg for msg in messages
                        if msg.get_srcSystem() == self.sysid_thismav() and msg.get_srcComponent() == 1 and
                        (message_type is None or msg.get_type() == message_type)]

            # Force the event-driven HOME/ORIGIN broadcasts after the links
            # exist, rather than depending on when startup GPS settles.
            self.wait_ekf_happy()
            collect()
            here = self.get_location()
            self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                             x=int(here.lat * 1e7), y=int(here.lng * 1e7), z=here.get_alt_m(AltFrame.ABSOLUTE),
                             frame=mavutil.mavlink.MAV_FRAME_GLOBAL)
            self.progress("Checking unicast links do not start normal telemetry streams")
            received = collect(12)
            normal_types = {msg.get_type() for msg in vehicle_messages(received["normal"])}
            default_types = {'EKF_STATUS_REPORT', 'ATTITUDE', 'GLOBAL_POSITION_INT', 'HOME_POSITION', 'GPS_GLOBAL_ORIGIN'}
            if not default_types.issubset(normal_types):
                raise NotAchievedException("Normal-link stream control failed: %s" % normal_types)
            for name in ("device", "second"):
                types = {msg.get_type() for msg in vehicle_messages(received[name])}
                if types != {'HEARTBEAT'}:
                    raise NotAchievedException("Unexpected default unicast messages on %s: %s" % (name, types))

            def request_device_message(command, message_id, interval=0):
                links["device"].mav.command_long_send(
                    self.sysid_thismav(), 1, command, 0, message_id, interval, 0, 0, 0, 0, 0)
                received = collect(2)
                if not any(msg.command == command and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                           for msg in vehicle_messages(received["device"], 'COMMAND_ACK')):
                    raise NotAchievedException("Unicast message request was not accepted: %u" % command)
                return received

            self.progress("Checking one-shot and explicitly requested telemetry on a unicast link")
            for message_type in ('HOME_POSITION', 'GPS_GLOBAL_ORIGIN'):
                message_id = getattr(mavutil.mavlink, 'MAVLINK_MSG_ID_' + message_type)
                received = request_device_message(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, message_id)
                if len(vehicle_messages(received['device'], message_type)) != 1:
                    raise NotAchievedException("Unicast one-shot %s request failed" % message_type)
                if vehicle_messages(received['second'], message_type):
                    raise NotAchievedException("Unicast %s reply leaked to another device" % message_type)
            ekf_message = mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT
            received = request_device_message(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, ekf_message)
            if len(vehicle_messages(received["device"], 'EKF_STATUS_REPORT')) != 1:
                raise NotAchievedException("Unicast one-shot request did not send exactly one EKF status")
            received = request_device_message(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, ekf_message, 200000)
            if len(vehicle_messages(received["device"], 'EKF_STATUS_REPORT')) < 5:
                raise NotAchievedException("Unicast did not start the requested EKF status stream")
            if vehicle_messages(received["second"], 'EKF_STATUS_REPORT'):
                raise NotAchievedException("Requested stream leaked onto another unicast link")
            # Reset-to-default must return to silence despite the non-zero
            # saved stream rate; it must not silently restart normal streams.
            request_device_message(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, ekf_message, 0)
            received = collect(2)
            if vehicle_messages(received["device"], 'EKF_STATUS_REPORT'):
                raise NotAchievedException("Reset-to-default restarted unicast EKF status")
            marker = 123456789
            links["device"].mav.timesync_send(0, marker)
            received = collect(1)
            if not any(msg.ts1 == marker and msg.tc1 != 0
                       for msg in vehicle_messages(received["device"], 'TIMESYNC')):
                raise NotAchievedException("Unicast suppressed a solicited time-sync reply")
            sequence = 0

            def check_forwarding(source, message, expected, bad_crc=False):
                nonlocal sequence
                sequence += 1
                self.progress("Routing check %u: %s %s -> %s" %
                              (sequence, source, message.get_type(), expected))
                collect(0.2)
                if bad_crc:
                    wire = bytearray(message.pack(links[source].mav))
                    wire[-1] ^= 1
                    links[source].mav.seq = (links[source].mav.seq + 1) % 256
                    links[source].write(wire)
                else:
                    links[source].mav.send(message)
                    wire = bytes(message.get_msgbuf())
                received = collect()
                counts = {name: sum(bytes(msg.get_msgbuf()) == wire for msg in messages)
                          for name, messages in received.items()}
                want = {name: int(name in expected) for name in links}
                if counts != want:
                    raise NotAchievedException("Routing %s from %s: got %s, want %s" %
                                               (message, source, counts, want))
                return received

            def ping(sysid, compid):
                return mavutil.mavlink.MAVLink_ping_message(123456, sequence, sysid, compid)

            self.progress("Checking broadcast isolation, including special heartbeat routing")
            for source, expected in (("gcs", ["normal"]), ("normal", ["gcs"]),
                                     ("device", []), ("second", []), ("private", [])):
                check_forwarding(source, mavutil.mavlink.MAVLink_heartbeat_message(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, sequence, 0, 3), expected)
                check_forwarding(source, mavutil.mavlink.MAVLink_attitude_message(
                    sequence, 0, 0, 0, 0, 0, 0), expected)
                check_forwarding(source, ping(0, 0), expected)
                check_forwarding(source, ping(0, 100), expected)
            # A system-addressed packet with a broadcast or absent component
            # must not enter a unicast link, even with a known system route.
            for sysid in (42, 43):
                check_forwarding("gcs", ping(sysid, 0), [])
                check_forwarding("gcs", mavutil.mavlink.MAVLink_set_mode_message(sysid, 0, 0), [])
            check_forwarding("device", ping(253, 0), [])
            check_forwarding("device", mavutil.mavlink.MAVLink_set_mode_message(253, 0, 0), [])

            self.progress("Checking exact routes and unchanged forwarded packet identities")
            check_forwarding("gcs", ping(42, 100), ["device"])
            check_forwarding("normal", ping(42, 100), ["device"])
            check_forwarding("gcs", ping(43, 101), ["second"])
            check_forwarding("gcs", ping(42, 101), [])
            check_forwarding("gcs", ping(45, 100), [])
            check_forwarding("device", ping(253, 191), ["normal"])
            check_forwarding("device", ping(self.mav.mav.srcSystem, self.mav.mav.srcComponent), ["gcs"])
            check_forwarding("device", ping(43, 101), ["second"])
            check_forwarding("second", ping(42, 100), ["device"])
            check_forwarding("device", ping(42, 100), [])  # no reflection

            # A malformed sender using component zero must not turn a
            # component broadcast into a unicast route.
            links["device"].mav.srcComponent = 0
            check_forwarding("device", mavutil.mavlink.MAVLink_heartbeat_message(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, sequence, 0, 3), [])
            check_forwarding("gcs", ping(42, 0), [])
            links["device"].mav.srcComponent = 100

            def check_extended_parameters(source):
                sysid = links[source].mav.srcSystem
                compid = links[source].mav.srcComponent
                param_id = b"CAM_TEST"
                value = (7).to_bytes(4, 'little')
                param_type = mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT32
                for request in (
                        mavutil.mavlink.MAVLink_param_ext_request_list_message(sysid, compid),
                        mavutil.mavlink.MAVLink_param_ext_request_read_message(sysid, compid, param_id, -1),
                        mavutil.mavlink.MAVLink_param_ext_set_message(sysid, compid, param_id, value, param_type)):
                    check_forwarding("gcs", request, [source])
                check_forwarding(source, mavutil.mavlink.MAVLink_param_ext_value_message(
                    param_id, value, param_type, 1, 0), [])
                for result in (mavutil.mavlink.PARAM_ACK_ACCEPTED,
                               mavutil.mavlink.PARAM_ACK_IN_PROGRESS,
                               mavutil.mavlink.PARAM_ACK_FAILED):
                    check_forwarding(source, mavutil.mavlink.MAVLink_param_ext_ack_message(
                        param_id, value, param_type, result), [])

            self.progress("Checking the router does not exempt extended parameter replies from broadcast isolation")
            check_extended_parameters("device")
            check_extended_parameters("second")
            # Replies on normal links are broadcasts too, and must not enter
            # isolated links. Broadcast requests must also remain isolated.
            check_forwarding("gcs", mavutil.mavlink.MAVLink_param_ext_request_list_message(0, 0), ["normal"])
            for reply in (
                    mavutil.mavlink.MAVLink_param_ext_value_message(
                        b"CAM_TEST", b"", mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT32, 1, 0),
                    mavutil.mavlink.MAVLink_param_ext_ack_message(
                        b"CAM_TEST", b"", mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT32,
                        mavutil.mavlink.PARAM_ACK_ACCEPTED)):
                check_forwarding("gcs", reply, ["normal"])
                check_forwarding("normal", reply, ["gcs"])

            self.progress("Checking legacy private-link behaviour, also with both options set")
            for options in (2, 18):
                self.set_parameter("MAV5_OPTIONS", options)
                # Private links do not forward addressed or broadcast replies.
                check_forwarding("gcs", ping(44, 102), ["private"])
                check_forwarding("private", ping(self.mav.mav.srcSystem, self.mav.mav.srcComponent), [])
                check_extended_parameters("private")

            self.progress("Checking bad-CRC forwarding cannot bypass unicast restrictions")
            self.set_parameters({"MAV1_OPTIONS": 8, "MAV3_OPTIONS": 24})
            check_forwarding("gcs", ping(0, 0), ["normal"], bad_crc=True)
            check_forwarding("device", ping(0, 0), [], bad_crc=True)
            check_forwarding("gcs", ping(42, 100), ["device"], bad_crc=True)
            check_forwarding("gcs", ping(42, 101), [], bad_crc=True)
            check_forwarding("device", ping(253, 191), ["normal"], bad_crc=True)

            # Also exercise a component sharing the flight controller's sysid.
            links["device"].mav.srcSystem = self.sysid_thismav()
            check_forwarding("device", mavutil.mavlink.MAVLink_heartbeat_message(
                mavutil.mavlink.MAV_TYPE_CAMERA, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, sequence, 0, 3), [])
            check_forwarding("gcs", ping(self.sysid_thismav(), 100), ["device"])
            check_extended_parameters("device")

            self.progress("Checking local processing and flight-controller-origin replies")
            for target in ((0, 0), (self.sysid_thismav(), 0), (self.sysid_thismav(), 1)):
                received = check_forwarding("device", mavutil.mavlink.MAVLink_param_request_read_message(
                    target[0], target[1], b"MAV_SYSID", -1), [])
                if not any(msg.get_type() == 'PARAM_VALUE' and msg.param_id == 'MAV_SYSID' and
                           msg.get_srcSystem() == self.sysid_thismav() and msg.get_srcComponent() == 1
                           for msg in received["device"]):
                    raise NotAchievedException("Unicast prevented local parameter reply for %s" % (target,))

            self.progress("Checking disabling unicast restores normal forwarding")
            self.set_parameter("MAV3_OPTIONS", 0)
            check_forwarding("device", ping(0, 0), ["gcs", "normal"])
            check_forwarding("normal", ping(0, 0), ["gcs", "device"])
        finally:
            mavutil.mavfile_global = saved_mavfile_global
            for name, link in links.items():
                if name != "gcs":
                    link.close()
