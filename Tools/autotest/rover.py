'''
Drive Rover in SITL

AP_FLAKE8_CLEAN

'''

import copy
import os
import pathlib
import statistics
import sys
import time

from pymavlink import mavutil

import vehicle_test_suite

from pysim import util
from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

SITL_START_LOCATION = Location(40.071374969556928,
                               -105.22978898137808,
                               1583.702759,
                               AltFrame.ABSOLUTE)
SITL_START_HEADING = 246


class AutoTestRover(vehicle_test_suite.TestSuite):
    @staticmethod
    def get_not_armable_mode_list():
        return ["RTL", "SMART_RTL"]

    @staticmethod
    def get_not_disarmed_settable_modes_list():
        return ["FOLLOW"]

    @staticmethod
    def get_no_position_not_settable_modes_list():
        return []

    @staticmethod
    def get_position_armable_modes_list():
        return ["GUIDED", "LOITER", "STEERING", "AUTO"]

    @staticmethod
    def get_normal_armable_modes_list():
        return ["ACRO", "HOLD", "MANUAL"]

    def log_name(self):
        return "Rover"

    def test_filepath(self):
        return os.path.realpath(__file__)

    test_root = "ArduRover_Tests"

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def sitl_start_heading(self):
        return SITL_START_HEADING

    def default_frame(self):
        return "rover"

    def default_speedup(self):
        return 30

    def is_rover(self):
        return True

    def get_stick_arming_channel(self):
        return int(self.get_parameter("RCMAP_ROLL"))

    def drive_left_circuit(self):
        """Drive a left circuit, 50m on a side."""
        self.change_mode('MANUAL')
        self.set_rc(3, 2000)

        self.progress("Driving left circuit")
        # do 4 turns
        for i in range(0, 4):
            # hard left
            self.progress("Starting turn %u" % i)
            self.set_rc(1, 1000)
            self.wait_heading(270 - (90*i), accuracy=10)
            self.set_rc(1, 1500)
            self.progress("Starting leg %u" % i)
            self.wait_distance(50, accuracy=7)
        self.set_rc(3, 1500)
        self.progress("Circuit complete")

    #################################################
    # AUTOTEST ALL
    #################################################
    def drive_mission(self, filename, strict=True, ignore_MANUAL_mode_change=False):
        """Drive a mission from a file."""
        self.progress("Driving mission %s" % filename)
        wp_count = self.load_mission(filename, strict=strict)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.wait_waypoint(1, wp_count-1, max_dist_to_final_wp_m=5, ignore_MANUAL_mode_change=ignore_MANUAL_mode_change)
        self.wait_statustext("Mission Complete", timeout=600)
        self.disarm_vehicle()
        self.progress("Mission OK")

    def drive_brake_get_stopping_distance(self, speed):
        '''measure our stopping distance'''

        self.context_push()

        # controller tends not to meet cruise speed (max of ~14 when 15
        # set), thus *1.2
        # at time of writing, the vehicle is only capable of 10m/s/s accel
        self.set_parameters({
            'CRUISE_SPEED': speed*1.2,
            'ATC_ACCEL_MAX': 15,
        })
        self.change_mode("STEERING")
        self.set_rc(3, 2000)
        self.wait_groundspeed(15, 100)
        initial = self.get_location()
        initial_time = time.time()
        while time.time() - initial_time < 2:
            # wait for a position update from the autopilot
            start = self.get_location()
            if start != initial:
                break
        self.set_rc(3, 1500)
        self.wait_groundspeed(0, 0.2)  # why do we not stop?!
        initial = self.get_location()
        initial_time = time.time()
        while time.time() - initial_time < 2:
            # wait for a position update from the autopilot
            stop = self.get_location()
            if stop != initial:
                break
        delta = self.get_distance(start, stop)

        self.context_pop()

        return delta

    def DriveBrake(self):
        '''Test braking'''
        self.set_parameters({
            'CRUISE_SPEED': 15,
            'ATC_BRAKE': 0,
        })

        self.arm_vehicle()

        distance_without_brakes = self.drive_brake_get_stopping_distance(15)

        # brakes on:
        self.set_parameter('ATC_BRAKE', 1)
        distance_with_brakes = self.drive_brake_get_stopping_distance(15)

        delta = distance_without_brakes - distance_with_brakes

        self.disarm_vehicle()

        if delta < distance_without_brakes * 0.05:  # 5% isn't asking for much
            raise NotAchievedException("""
Brakes have negligible effect (with=%0.2fm without=%0.2fm delta=%0.2fm)
""" %
                                       (distance_with_brakes,
                                        distance_without_brakes,
                                        delta))

        self.progress(
            "Brakes work (with=%0.2fm without=%0.2fm delta=%0.2fm)" %
            (distance_with_brakes, distance_without_brakes, delta))

    def test_gcs_fence_need_centroid(self, target_system=1, target_component=1):
        return copy.copy([
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                4, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                31.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                4, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0002 * 1e7), # latitude
                int(1.0000 * 1e7), # longitude
                32.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                4, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0002 * 1e7), # latitude
                int(1.0001 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
            self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                3, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                0, # current
                0, # autocontinue
                4, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.0000 * 1e7), # latitude
                int(1.0001 * 1e7), # longitude
                33.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_FENCE),
        ])

    def click_location_from_item(self, mavproxy, item):
        mavproxy.send("click %f %f\n" % (item.x*1e-7, item.y*1e-7))

#        MANUAL> usage: fence <addcircle|addpoly|changealt|clear|disable|draw|enable|list|load|move|movemulti|movepolypoint|param|remove|save|savecsv|savelocal|show|status|undo|update>  # noqa

    # explode the write_type_to_storage method
    # FIXME: test converting invalid fences / minimally valid fences / normal fences
    # FIXME: show that uploading smaller items take up less space
    # FIXME: add test for consecutive breaches within the manual recovery period
    # FIXME: ensure truncation does the right thing by fence_total

    def assert_mission_count_on_link(self, mav, expected_count, target_system, target_component, mission_type):
        self.drain_mav_unparsed(mav=mav, freshen_sim_time=True)
        self.progress("waiting for a message - any message....")
        m = mav.recv_match(blocking=True, timeout=1)
        self.progress("Received (%s)" % str(m))

        if not mav.mavlink20():
            raise NotAchievedException("Not doing mavlink2")
        mav.mav.mission_request_list_send(target_system,
                                          target_component,
                                          mission_type)
        self.assert_receive_mission_count_on_link(mav,
                                                  expected_count,
                                                  target_system,
                                                  target_component,
                                                  mission_type)

    def assert_receive_mission_count_on_link(self,
                                             mav,
                                             expected_count,
                                             target_system,
                                             target_component,
                                             mission_type,
                                             expected_target_system=None,
                                             expected_target_component=None,
                                             timeout=120):
        if expected_target_system is None:
            expected_target_system = mav.mav.srcSystem
        if expected_target_component is None:
            expected_target_component = mav.mav.srcComponent
        self.progress("Waiting for mission count of (%u) from (%u:%u) to (%u:%u)" %
                      (expected_count, target_system, target_component, expected_target_system, expected_target_component))

        tstart = self.get_sim_time_cached()
        while True:
            delta = self.get_sim_time_cached() - tstart
            if delta > timeout:
                raise NotAchievedException("Did not receive MISSION_COUNT on link after %fs" % delta)
            m = mav.recv_match(blocking=True, timeout=1)
            if m is None:
                self.progress("No messages")
                continue
#            self.progress("Received (%s)" % str(m))
            if m.get_type() == "MISSION_ACK":
                if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise NotAchievedException("Expected MAV_MISSION_ACCEPTED, got (%s)" % m)
            if m.get_type() == "MISSION_COUNT":
                break
        if m.target_system != expected_target_system:
            raise NotAchievedException("Incorrect target system in MISSION_COUNT (want=%u got=%u)" %
                                       (expected_target_system, m.target_system))
        if m.target_component != expected_target_component:
            raise NotAchievedException("Incorrect target component in MISSION_COUNT")
        if m.mission_type != mission_type:
            raise NotAchievedException("Did not get expected mission type (want=%u got=%u)" % (mission_type, m.mission_type))
        if m.count != expected_count:
            raise NotAchievedException("Bad count received (want=%u got=%u)" %
                                       (expected_count, m.count))
        self.progress("Asserted mission count (type=%u) is %u after %fs" % (
            (mission_type, m.count, delta)))

    def get_mission_item_int_on_link(self, item, mav, target_system, target_component, mission_type, delay_fn=None):
        self.drain_mav(mav=mav, unparsed=True)
        mav.mav.mission_request_int_send(target_system,
                                         target_component,
                                         item,
                                         mission_type)
        m = self.assert_receive_message(
            'MISSION_ITEM_INT',
            timeout=60,
            condition='MISSION_ITEM_INT.mission_type==%u' % mission_type,
            delay_fn=delay_fn)
        if m is None:
            raise NotAchievedException("Did not receive MISSION_ITEM_INT")
        if m.mission_type != mission_type:
            raise NotAchievedException("Mission item of incorrect type")
        if m.target_system != mav.mav.srcSystem:
            raise NotAchievedException("Unexpected target system %u want=%u" %
                                       (m.target_system, mav.mav.srcSystem))
        if m.seq != item:
            raise NotAchievedException(
                "Incorrect sequence number on received item got=%u want=%u" %
                (m.seq, item))
        if m.mission_type != mission_type:
            raise NotAchievedException(
                "Mission type incorrect on received item (want=%u got=%u)" %
                (mission_type, m.mission_type))
        if m.target_component != mav.mav.srcComponent:
            raise NotAchievedException(
                "Unexpected target component %u want=%u" %
                (m.target_component, mav.mav.srcComponent))
        return m

    def get_mission_item_on_link(self, item, mav, target_system, target_component, mission_type):
        self.drain_mav(mav=mav, unparsed=True)
        mav.mav.mission_request_send(target_system,
                                     target_component,
                                     item,
                                     mission_type)
        m = self.assert_receive_message('MISSION_ITEM', timeout=60)
        if m.target_system != mav.mav.srcSystem:
            raise NotAchievedException("Unexpected target system %u want=%u" %
                                       (m.target_system, mav.mav.srcSystem))
        if m.seq != item:
            raise NotAchievedException("Incorrect sequence number on received item_int got=%u want=%u" %
                                       (m.seq, item))
        if m.mission_type != mission_type:
            raise NotAchievedException("Mission type incorrect on received item_int (want=%u got=%u)" %
                                       (mission_type, m.mission_type))
        if m.target_component != mav.mav.srcComponent:
            raise NotAchievedException("Unexpected target component %u want=%u" %
                                       (m.target_component, mav.mav.srcComponent))
        return m

    def assert_receive_mission_item_request(self, mission_type, seq):
        self.progress("Expecting request for item %u" % seq)
        m = self.assert_receive_message('MISSION_REQUEST')
        if m.mission_type != mission_type:
            raise NotAchievedException("Incorrect mission type (wanted=%u got=%u)" %
                                       (mission_type, m.mission_type))
        if m.seq != seq:
            raise NotAchievedException("Unexpected sequence number (want=%u got=%u)" % (seq, m.seq))
        self.progress("Received item request OK")

    def assert_receive_mission_ack(self, mission_type,
                                   want_type=mavutil.mavlink.MAV_MISSION_ACCEPTED,
                                   target_system=None,
                                   target_component=None,
                                   mav=None):
        if mav is None:
            mav = self.mav
        if target_system is None:
            target_system = mav.mav.srcSystem
        if target_component is None:
            target_component = mav.mav.srcComponent
        self.progress("Expecting mission ack")
        m = self.assert_receive_message(
            'MISSION_ACK',
            timeout=5,
            verbose=True,
        )
        if m.target_system != target_system:
            raise NotAchievedException("ACK not targeted at correct system want=%u got=%u" %
                                       (target_system, m.target_system))
        if m.target_component != target_component:
            raise NotAchievedException("ACK not targeted at correct component")
        if m.mission_type != mission_type:
            raise NotAchievedException("Unexpected mission type %u want=%u" %
                                       (m.mission_type, mission_type))
        if m.type != want_type:
            raise NotAchievedException("Expected ack type %u got %u" %
                                       (want_type, m.type))
        return m

    def assert_filepath_content(self, filepath, want):
        got = pathlib.Path(filepath).read_text()
        if want != got:
            raise NotAchievedException("Did not get expected file content (want=%s) (got=%s)" % (want, got))

    def mavproxy_can_do_mision_item_protocols(self):
        return False
        if not self.mavproxy_version_gt(1, 8, 69):
            self.progress("MAVProxy is too old; skipping tests")
            return False
        return True

    def check_rally_items_same(self, want, got, epsilon=None):
        check_atts = ['mission_type', 'command', 'x', 'y', 'z', 'seq', 'param1']
        return self.check_mission_items_same('rally', check_atts, want, got, epsilon=epsilon)

    def click_three_in(self, mavproxy, target_system=1, target_component=1):
        mavproxy.send('rally clear\n')
        self.drain_mav()
        # there are race conditions in MAVProxy.  Beware.
        mavproxy.send("click 1.0 1.0\n")
        mavproxy.send("rally add\n")
        self.delay_sim_time(1, reason="rally upload to propagate")
        mavproxy.send("click 2.0 2.0\n")
        mavproxy.send("rally add\n")
        self.delay_sim_time(1, reason="rally upload to propagate")
        mavproxy.send("click 3.0 3.0\n")
        mavproxy.send("rally add\n")
        self.delay_sim_time(10, reason="rally upload to complete")
        self.assert_mission_count_on_link(
            self.mav,
            3,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )

    def GCSRally(self, target_system=1, target_component=1):
        '''Upload and download of rally using MAVProxy'''
        self.start_subtest("Testing mavproxy CLI for rally points")
        mavproxy = self.start_mavproxy()

        rallyalt = 87  # note that this exists as a string in expected results below
        mavproxy.send(f'set rallyalt {rallyalt}\n')
        mavproxy.send('rally clear\n')

        self.start_subsubtest("rally add")
        mavproxy.send('rally clear\n')
        lat_s = "-5.6789"
        lng_s = "98.2341"
        lat = float(lat_s)
        lng = float(lng_s)
        mavproxy.send(f'click {lat_s} {lng_s}\n')
        self.drain_mav()
        mavproxy.send('rally add\n')
        self.delay_sim_time(2, reason="rally upload to propagate")
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 1:
            raise NotAchievedException("Unexpected count (got=%u want=1)" %
                                       (len(downloaded_items), ))
        if abs(downloaded_items[0].x - int(lat * 1e7)) > 1:
            raise NotAchievedException("Bad rally lat.  Want=%d got=%d" %
                                       (int(lat * 1e7), downloaded_items[0].x))
        if abs(downloaded_items[0].y - int(lng * 1e7)) > 1:
            raise NotAchievedException("Bad rally lng.  Want=%d got=%d" %
                                       (int(lng * 1e7), downloaded_items[0].y))
        if abs(downloaded_items[0].z - rallyalt) > 1:
            raise NotAchievedException(f"Bad rally alt.  Want={rallyalt} got={downloaded_items[0].z}")
        self.end_subsubtest("rally add")

        self.start_subsubtest("rally list")
        util.pexpect_drain(mavproxy)
        mavproxy.send('rally list\n')
        mavproxy.expect(r"Saved 1 rally items to ([^\s]*)\s")
        filename = mavproxy.match.group(1)
        self.assert_rally_filepath_content(filename, '''QGC WPL 110
0	0	3	5100	0.000000	0.000000	0.000000	0.000000	-5.678900	98.234100	87.000000	0
''')
        self.end_subsubtest("rally list")

        self.start_subsubtest("rally save")
        util.pexpect_drain(mavproxy)
        save_tmppath = self.buildlogs_path("rally-testing-tmp.txt")
        mavproxy.send('rally save %s\n' % save_tmppath)
        mavproxy.expect(r"Saved 1 rally items to ([^\s]*)\s")
        filename = mavproxy.match.group(1)
        if filename != save_tmppath:
            raise NotAchievedException("Bad save filepath; want=%s got=%s" % (save_tmppath, filename))
        self.assert_rally_filepath_content(filename, '''QGC WPL 110
0	0	3	5100	0.000000	0.000000	0.000000	0.000000	-5.678900	98.234100	87.000000	0
''')
        self.end_subsubtest("rally save")

        self.start_subsubtest("rally savecsv")
        util.pexpect_drain(mavproxy)
        csvpath = self.buildlogs_path("rally-testing-tmp.csv")
        mavproxy.send('rally savecsv %s\n' % csvpath)
        (major, minor, micro, releaselevel, serial) = sys.version_info
        if major == 3 and minor == 12:
            # greater precision output by default
            expected_content = '''"0","Rel","NAV_RALLY_POINT","0.0","0.0","0.0","0.0","-5.678899899999999","98.2341","87.0"
'''  # noqa
        # wait for the file to appear:
        for i in range(1, 10):
            if os.path.exists(csvpath):
                break
            time.sleep(0.1)
        # file content does not seem stable, even within a Python major version
        self.assert_filepath_content(csvpath, expected_content)

        self.end_subsubtest("rally savecsv")

        self.start_subsubtest("rally load")
        self.drain_mav()
        mavproxy.send('rally clear\n')
        self.wait_heartbeat()
        self.wait_heartbeat()
        self.assert_mission_count_on_link(self.mav,
                                          0,
                                          target_system,
                                          target_component,
                                          mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

        # warning: uses file saved from previous test
        self.start_subtest("Check rally load from filepath")
        mavproxy.send('rally load %s\n' % save_tmppath)
        mavproxy.expect(r"Loaded 1 rally items from ([^\s]*)\s")
        mavproxy.expect("Sent all .* rally items") # notional race condition here
        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 1:
            raise NotAchievedException("Unexpected item count (%u)" % len(downloaded_items))
        if abs(int(downloaded_items[0].x) - int(lat * 1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" %
                                       (lat * 1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[0].y) - int(lng * 1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" %
                                       (lng * 1e7, downloaded_items[0].y))
        self.end_subsubtest("rally load")

        self.start_subsubtest("rally changealt")
        mavproxy.send('rally clear\n')
        mavproxy.send("click 1.0 1.0\n")
        mavproxy.send("rally add\n")
        mavproxy.send("click 2.0 2.0\n")
        mavproxy.send("rally add\n")
        self.delay_sim_time(2, reason="rally upload to propagate")
        self.assert_mission_count_on_link(
            self.mav,
            2,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        self.drain_mav()
        mavproxy.send("rally changealt 1 17.6\n")
        self.wait_heartbeat()
        self.wait_heartbeat()
        mavproxy.send("rally changealt 2 19.1\n")
        self.delay_sim_time(2, reason="rally alt change to propagate")

        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 2:
            raise NotAchievedException("Unexpected item count (%u)" % len(downloaded_items))
        if abs(int(downloaded_items[0].x) - int(1 * 1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" % (1 * 1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[0].y) - int(1 * 1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" % (1 * 1e7, downloaded_items[0].y))
        # at some stage ArduPilot will stop rounding altitude.  This
        # will break then.
        if abs(downloaded_items[0].z - int(17.6)) > 0.0001:
            raise NotAchievedException("Expected alt=%f got=%f" % (17.6, downloaded_items[0].z))

        if abs(int(downloaded_items[1].x) - int(2 * 1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" % (2 * 1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[1].y) - int(2 * 1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" % (2 * 1e7, downloaded_items[0].y))
        # at some stage ArduPilot will stop rounding altitude.  This
        # will break then.
        if abs(downloaded_items[1].z - int(19.1)) > 0.0001:
            raise NotAchievedException("Expected alt=%f got=%f" % (19.1, downloaded_items[1].z))

        self.progress("Now change two at once")
        mavproxy.send("rally changealt 1 17.3 2\n")
        self.delay_sim_time(2, reason="rally alt change to propagate")
        mavproxy.send('rally list\n')
        self.drain_mav()
        self.wait_heartbeat()

        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 2:
            raise NotAchievedException("Unexpected item count (%u)" % len(downloaded_items))

        if abs(int(downloaded_items[0].x) - int(1 * 1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" % (1 * 1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[0].y) - int(1 * 1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" % (1 * 1e7, downloaded_items[0].y))
        # at some stage ArduPilot will stop rounding altitude.  This
        # will break then.
        if abs(int(downloaded_items[0].z) - int(17.3)) > 0.0001:
            raise NotAchievedException("Expected alt=%f got=%f" % (17.3, downloaded_items[0].z))

        if abs(int(downloaded_items[1].x) - int(2 * 1e7)) > 3:
            raise NotAchievedException("Expected lat=%d got=%d" % (2 * 1e7, downloaded_items[0].x))
        if abs(int(downloaded_items[1].y) - int(2 * 1e7)) > 10:
            raise NotAchievedException("Expected lng=%d got=%d" % (2 * 1e7, downloaded_items[0].y))
        # at some stage ArduPilot will stop rounding altitude.  This
        # will break then.
        if abs(int(downloaded_items[1].z) - int(17.3)) > 0.0001:
            raise NotAchievedException("Expected alt=%f got=%f" % (17.3, downloaded_items[0].z))

        self.end_subsubtest("rally changealt")

        self.start_subsubtest("rally move")
        mavproxy.send('rally clear\n')
        mavproxy.send("click 1.0 1.0\n")
        mavproxy.send("rally add\n")
        mavproxy.send("click 2.0 2.0\n")
        mavproxy.send("rally add\n")
        self.delay_sim_time(2, reason="rally upload to propagate")
        self.assert_mission_count_on_link(
            self.mav,
            2,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        mavproxy.send("click 3.0 3.0\n")
        mavproxy.send("rally move 2\n")
        self.delay_sim_time(2, reason="rally move to propagate")
        mavproxy.send("click 4.12345 4.987654\n")
        mavproxy.send("rally move 1\n")
        self.delay_sim_time(2, reason="rally move to propagate")

        downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(downloaded_items) != 2:
            raise NotAchievedException("Unexpected item count (%u)" % len(downloaded_items))
        if downloaded_items[0].x != 41234500:
            raise NotAchievedException("Bad latitude")
        if downloaded_items[0].y != 49876540:
            raise NotAchievedException("Bad longitude")
        if downloaded_items[0].z != rallyalt:
            raise NotAchievedException("Bad altitude (want=%u got=%u)" %
                                       (90, downloaded_items[0].z))

        if downloaded_items[1].x != 30000000:
            raise NotAchievedException("Bad latitude")
        if downloaded_items[1].y != 30000000:
            raise NotAchievedException("Bad longitude")
        if downloaded_items[1].z != rallyalt:
            raise NotAchievedException("Bad altitude (want=%u got=%u)" %
                                       (rallyalt, downloaded_items[1].z))
        self.end_subsubtest("rally move")

        # no "rally movemulti"> test for rally here
        # no "rally param" test for rally here

        self.start_subsubtest("rally remove")
        self.click_three_in(mavproxy, target_system=target_system, target_component=target_component)
        pure_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.progress("Removing last in list")
        mavproxy.send("rally remove 3\n")
        self.delay_sim_time(2, reason="rally remove to propagate")
        self.assert_mission_count_on_link(
            self.mav,
            2,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        fewer_downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(fewer_downloaded_items) != 2:
            raise NotAchievedException("Unexpected download list length")
        shorter_items = copy.copy(pure_items)
        shorter_items = shorter_items[0:2]
        self.check_rally_items_same(shorter_items, fewer_downloaded_items)

        self.progress("Removing first in list")
        mavproxy.send("rally remove 1\n")
        self.delay_sim_time(2, reason="rally remove to propagate")
        self.assert_mission_count_on_link(
            self.mav,
            1,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        fewer_downloaded_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        if len(fewer_downloaded_items) != 1:
            raise NotAchievedException("Unexpected download list length")
        shorter_items = shorter_items[1:]
        self.check_rally_items_same(shorter_items, fewer_downloaded_items)

        self.progress("Removing remaining item")
        mavproxy.send("rally remove 1\n")
        self.delay_sim_time(2, reason="rally remove to propagate")
        self.assert_mission_count_on_link(
            self.mav,
            0,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        self.end_subsubtest("rally remove")

        self.start_subsubtest("rally show")
        # what can we test here?
        mavproxy.send("rally show %s\n" % save_tmppath)
        self.end_subsubtest("rally show")

        # savelocal must be run immediately after show!
        self.start_subsubtest("rally savelocal")
        util.pexpect_drain(mavproxy)
        savelocal_path = self.buildlogs_path("rally-testing-tmp-local.txt")
        mavproxy.send('rally savelocal %s\n' % savelocal_path)
        self.delay_sim_time(2, reason="file save to complete")
        self.assert_rally_filepath_content(savelocal_path, '''QGC WPL 110
0	0	3	5100	0.000000	0.000000	0.000000	0.000000	-5.678900	98.234100	87.000000	0
''')
        self.end_subsubtest("rally savelocal")

        self.start_subsubtest("rally status")
        self.click_three_in(mavproxy, target_system=target_system, target_component=target_component)
        mavproxy.send("rally status\n")
        mavproxy.expect("Have 3 of 3 rally items")
        mavproxy.send("rally clear\n")
        mavproxy.send("rally status\n")
        mavproxy.expect("Have 0 of 0 rally items")
        self.end_subsubtest("rally status")

        self.start_subsubtest("rally undo")
        self.progress("Testing undo-remove")
        self.click_three_in(mavproxy, target_system=target_system, target_component=target_component)
        pure_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.progress("Removing first in list")
        mavproxy.send("rally remove 1\n")
        self.delay_sim_time(2, reason="rally remove to propagate")
        self.assert_mission_count_on_link(
            self.mav,
            2,
            target_system,
            target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
        )
        mavproxy.send("rally undo\n")
        self.delay_sim_time(2, reason="rally undo to propagate")
        undone_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.check_rally_items_same(pure_items, undone_items)

        self.progress("Testing undo-move")

        pure_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        mavproxy.send("click 4.12345 4.987654\n")
        mavproxy.send("rally move 1\n")
        # move has already been tested, assume it works...
        self.delay_sim_time(2, reason="rally move to propagate")
        mavproxy.send("rally undo\n")
        self.delay_sim_time(2, reason="rally undo to propagate")
        undone_items = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
        self.check_rally_items_same(pure_items, undone_items)

        self.end_subsubtest("rally undo")

        # no tests for "rally update" here

        self.stop_mavproxy(mavproxy)

# MANUAL> usage: rally <add|alt|changealt|clear|list|load|move|movemulti|param|remove|save|savecsv|savelocal|show|status|undo|update>  # noqa

    def RallyUploadDownload(self, target_system=1, target_component=1):
        '''Upload and download of rally'''
        old_srcSystem = self.mav.mav.srcSystem

        self.drain_mav()
        try:
            item1_lat = int(2.0000 * 1e7)
            items = [
                self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    0, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                    0, # current
                    0, # autocontinue
                    0, # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    int(1.0000 * 1e7), # latitude
                    int(1.0000 * 1e7), # longitude
                    31.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
                self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    1, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                    0, # current
                    0, # autocontinue
                    0, # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    item1_lat, # latitude
                    int(2.0000 * 1e7), # longitude
                    32.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
                self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    2, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                    0, # current
                    0, # autocontinue
                    0, # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    int(3.0000 * 1e7), # latitude
                    int(3.0000 * 1e7), # longitude
                    33.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            ]
            self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                               items)
            downloaded = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            print("Got items (%s)" % str(items))
            if len(downloaded) != len(items):
                raise NotAchievedException(
                    "Did not download correct number of items want=%u got=%u" %
                    (len(downloaded), len(items)))

            rally_total = self.get_parameter("RALLY_TOTAL")
            if rally_total != len(downloaded):
                raise NotAchievedException(
                    "Unexpected rally point count: want=%u got=%u" %
                    (len(items), rally_total))

            self.progress("Pruning count by setting parameter (urgh)")
            self.set_parameter("RALLY_TOTAL", 2)
            downloaded = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            if len(downloaded) != 2:
                raise NotAchievedException(
                    "Failed to prune rally points by setting parameter.  want=%u got=%u" %
                    (2, len(downloaded)))

            self.progress("Uploading a third item using old protocol")
            new_item2_lat = int(6.0 * 1e7)
            self.set_parameter("RALLY_TOTAL", 3)
            self.mav.mav.rally_point_send(target_system,
                                          target_component,
                                          2, # sequence number
                                          3, # total count
                                          new_item2_lat,
                                          int(7.0 * 1e7),
                                          15,
                                          0, # "break" alt?!
                                          0, # "land dir"
                                          0) # flags
            downloaded = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            if len(downloaded) != 3:
                raise NotAchievedException(
                    "resetting rally point count didn't change items returned")
            if downloaded[2].x != new_item2_lat:
                raise NotAchievedException(
                    "Bad latitude in downloaded item: want=%u got=%u" %
                    (new_item2_lat, downloaded[2].x))

            self.progress("Grabbing original item 1 using original protocol")
            self.mav.mav.rally_fetch_point_send(target_system,
                                                target_component,
                                                1)
            m = self.assert_receive_message("RALLY_POINT")
            if m.target_system != self.mav.source_system:
                raise NotAchievedException(
                    "Bad target_system on received rally point (want=%u got=%u)" %
                    (255, m.target_system))
            if m.target_component != self.mav.source_component: # autotest's component ID
                raise NotAchievedException("Bad target_component on received rally point")
            if m.lat != item1_lat:
                raise NotAchievedException("Bad latitude on received rally point")

            self.start_subtest("Test upload lockout and timeout")
            self.progress("Starting upload from normal sysid")
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            len(items),
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.drain_mav() # throw away requests for items
            self.mav.mav.srcSystem = 243

            self.progress("Attempting upload from sysid=%u" %
                          (self.mav.mav.srcSystem,))
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            len(items),
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_DENIED)

            self.progress("Attempting download from sysid=%u" %
                          (self.mav.mav.srcSystem,))
            self.mav.mav.mission_request_list_send(target_system,
                                                   target_component,
                                                   mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_DENIED)

            # wait for the upload from sysid=1 to time out:
            tstart = self.get_sim_time()
            got_statustext = False
            got_ack = False
            while True:
                if got_statustext and got_ack:
                    self.progress("Got both ack and statustext")
                    break
                if self.get_sim_time_cached() - tstart > 100:
                    raise NotAchievedException("Did not get both ack and statustext")
                m = self.mav.recv_match(type=['STATUSTEXT', 'MISSION_ACK'],
                                        blocking=True,
                                        timeout=1)
                if m is None:
                    continue
                self.progress("Got (%s)" % str(m))
                if m.get_type() == 'STATUSTEXT':
                    if "upload timeout" in m.text:
                        got_statustext = True
                        self.progress("Received desired statustext")
                    continue
                if m.get_type() == 'MISSION_ACK':
                    if m.target_system != old_srcSystem:
                        raise NotAchievedException("Incorrect sourcesystem")
                    if m.type != mavutil.mavlink.MAV_MISSION_OPERATION_CANCELLED:
                        raise NotAchievedException("Incorrect result")
                    if m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_RALLY:
                        raise NotAchievedException("Incorrect mission_type")
                    got_ack = True
                    self.progress("Received desired ACK")
                    continue
                raise NotAchievedException("Huh?")

            self.progress("Now trying to upload empty mission after timeout")
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            0,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.drain_mav()
            self.start_subtest("Check rally upload/download across separate links")
            self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                               items)
            self.progress("ensure a mavlink1 connection can't do anything useful with new item types")
            self.set_parameter("SERIAL2_PROTOCOL", 1)
            self.reboot_sitl()
            mav2 = mavutil.mavlink_connection(self.sitl_serial_endpoint(2),
                                              robust_parsing=True,
                                              source_system=7,
                                              source_component=7)
            mav2.mav.mission_request_list_send(target_system,
                                               target_component,
                                               mission_type=mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            # so this looks a bit odd; the other end isn't sending
            # mavlink2 so can't fill in the extension here.
            self.assert_receive_mission_ack(
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                want_type=mavutil.mavlink.MAV_MISSION_UNSUPPORTED,
                mav=mav2,
            )
            # this relies on magic upgrade to serial2:
            self.set_parameter("SERIAL2_PROTOCOL", 2)
            expected_count = 3
            self.progress("Assert mission count on new link")
            self.assert_mission_count_on_link(
                mav2,
                expected_count,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.progress("Assert mission count on original link")
            self.assert_mission_count_on_link(
                self.mav,
                expected_count,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.progress("Get first item on new link")

            def drain_self_mav_fn():
                self.drain_mav(self.mav)
            m2 = self.get_mission_item_int_on_link(
                2,
                mav2,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                delay_fn=drain_self_mav_fn)
            self.progress("Get first item on original link")
            m = self.get_mission_item_int_on_link(
                2,
                self.mav,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            if m2.x != m.x:
                raise NotAchievedException("mission items do not match (%d vs %d)" % (m2.x, m.x))
            self.get_mission_item_on_link(2, self.mav, target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            # ensure we get nacks for bad mission item requests:
            self.mav.mav.mission_request_send(target_system,
                                              target_component,
                                              65,
                                              mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE,
            )
            self.mav.mav.mission_request_int_send(target_system,
                                                  target_component,
                                                  65,
                                                  mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE,
            )

            self.start_subtest("Should enforce items come from correct GCS")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            1,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 0)
            self.progress("Attempting to upload from bad sysid")
            old_sysid = self.mav.mav.srcSystem
            self.mav.mav.srcSystem = 17
            items[0].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[0])
            self.mav.mav.srcSystem = old_sysid
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_DENIED,
                                            target_system=17)
            self.progress("Sending from correct sysid")
            items[0].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[0])
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.drain_mav()
            self.drain_all_pexpects()

            self.start_subtest("Attempt to send item on different link to that which we are sending requests on")
            self.progress("Sending count")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            2,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 0)
            old_mav2_system = mav2.mav.srcSystem
            old_mav2_component = mav2.mav.srcComponent
            mav2.mav.srcSystem = self.mav.mav.srcSystem
            mav2.mav.srcComponent = self.mav.mav.srcComponent
            self.progress("Sending item on second link")
            # note that the routing table in ArduPilot now will say
            # this sysid/compid is on both links which may cause
            # weirdness...
            items[0].pack(mav2.mav)
            self.drain_mav(mav=self.mav, unparsed=True)
            mav2.mav.send(items[0])
            mav2.mav.srcSystem = old_mav2_system
            mav2.mav.srcComponent = old_mav2_component
            # we continue to receive requests on the original link:
            m = self.assert_receive_message('MISSION_REQUEST')
            if m.mission_type != mavutil.mavlink.MAV_MISSION_TYPE_RALLY:
                raise NotAchievedException("Mission request of incorrect type")
            if m.seq != 1:
                raise NotAchievedException("Unexpected sequence number (expected=%u got=%u)" % (1, m.seq))
            items[1].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[1])
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.drain_mav()
            self.drain_all_pexpects()

            self.start_subtest("Upload mission and rally points at same time")
            self.progress("Sending rally count")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            3,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 0)

            self.progress("Sending wp count")
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            3,
                                            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_MISSION, 0)

            self.progress("Answering request for mission item 0")
            self.drain_mav(mav=self.mav, unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.1000 * 1e7), # latitude
                int(1.2000 * 1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_MISSION, 1)

            self.progress("Answering request for rally point 0")
            items[0].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[0])
            self.progress("Expecting request for rally item 1")
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 1)
            self.progress("Answering request for rally point 1")
            items[1].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[1])
            self.progress("Expecting request for rally item 2")
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 2)

            self.progress("Answering request for rally point 2")
            items[2].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[2])
            self.progress("Expecting mission ack for rally")
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)

            self.progress("Answering request for waypoints item 1")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.1000 * 1e7), # latitude
                int(1.2000 * 1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_MISSION, 2)

            self.progress("Answering request for waypoints item 2")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.1000 * 1e7), # latitude
                int(1.2000 * 1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

            self.start_subtest("Test write-partial-list")
            self.progress("Clearing rally points using count-send")
            self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                               target_system=target_system,
                               target_component=target_component)
            self.progress("Should not be able to set items completely past the waypoint count")
            self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                               items)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_write_partial_list_send(
                target_system,
                target_component,
                17,
                20,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_ERROR)

            self.progress("Should not be able to set items overlapping the waypoint count")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_write_partial_list_send(
                target_system,
                target_component,
                0,
                20,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_ERROR)

            self.progress("try to overwrite items 1 and 2")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_write_partial_list_send(
                target_system,
                target_component,
                1,
                2,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 1)
            self.progress("Try shoving up an incorrectly sequenced item")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.1000 * 1e7), # latitude
                int(1.2000 * 1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE)

            self.progress("Try shoving up an incorrectly sequenced item (but within band)")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                2, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(1.1000 * 1e7), # latitude
                int(1.2000 * 1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE)

            self.progress("Now provide correct item")
            item1_latitude = int(1.2345 * 1e7)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_item_int_send(
                target_system,
                target_component,
                1, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,
                0, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                item1_latitude, # latitude
                int(1.2000 * 1e7), # longitude
                321.0000, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY),
            self.assert_receive_mission_item_request(mavutil.mavlink.MAV_MISSION_TYPE_RALLY, 2)
            self.progress("Answering request for rally point 2")
            items[2].pack(self.mav.mav)
            self.drain_mav(unparsed=True)
            self.mav.mav.send(items[2])
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.progress("TODO: ensure partial mission write was good")

            self.start_subtest("clear mission types")
            self.assert_mission_count_on_link(
                self.mav,
                3,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(
                self.mav,
                3,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_clear_all_send(target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(
                self.mav,
                0,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(
                self.mav,
                3,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_clear_all_send(target_system, target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            self.assert_mission_count_on_link(
                self.mav,
                0,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_mission_count_on_link(
                self.mav,
                0,
                target_system,
                target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

            self.start_subtest("try sending out-of-range counts")
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            1,
                                            230)
            self.assert_receive_mission_ack(230,
                                            want_type=mavutil.mavlink.MAV_MISSION_UNSUPPORTED)
            self.drain_mav(unparsed=True)
            self.mav.mav.mission_count_send(target_system,
                                            target_component,
                                            16000,
                                            mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.assert_receive_mission_ack(mavutil.mavlink.MAV_MISSION_TYPE_RALLY,
                                            want_type=mavutil.mavlink.MAV_MISSION_NO_SPACE)

        except Exception as e:
            self.progress("Received exception (%s)" % self.get_exception_stacktrace(e))
            self.mav.mav.srcSystem = old_srcSystem
            raise e
        self.reboot_sitl()

    def drive_to_location(self, loc, tolerance=1, timeout=30, target_system=1, target_component=1):
        self.assert_mode('GUIDED')

        type_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE +
                     mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

        last_sent = 0
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException("Did not get to location")
            if now - last_sent > 10:
                last_sent = now
                self.mav.mav.set_position_target_global_int_send(
                    0,
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    type_mask,
                    int(loc.lat * 1.0e7),
                    int(loc.lng * 1.0e7),
                    0, # alt
                    0, # x-ve
                    0, # y-vel
                    0, # z-vel
                    0, # afx
                    0, # afy
                    0, # afz
                    0, # yaw,
                    0, # yaw-rate
                )
            if self.get_distance(self.get_location(), loc) > tolerance:
                continue
            return

    def wait_rtl_complete(self):
        """Wait for RTL to reach home and disarm"""
        self.progress("Waiting RTL to reach Home")
        self.wait_distance_to_home(0, 7, timeout=30)

    def send_guided_mission_item(self, loc, target_system=1, target_component=1):
        self.mav.mav.mission_item_send(
            target_system,
            target_component,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, # current
            0, # autocontinue
            0, # param1
            0, # param2
            0, # param3
            0, # param4
            loc.lat, # x
            loc.lng, # y
            0 # z
        )

    def do_RTL(self, distance_min=3, distance_max=7, timeout=60):
        self.change_mode("RTL")
        self.wait_distance_to_home(distance_min, distance_max, timeout=timeout)

    def mavlink_time_boot_ms(self):
        '''returns a time suitable for putting into the time_boot_ms entry in mavlink packets'''
        return int(time.time() * 1000000)

    def assert_mode(self, mode):
        if not self.mode_is(mode):
            raise NotAchievedException("Mode is not %s" % str(mode))

    def start_driving_simple_relhome_mission(self, items):
        '''uploads items, changes mode to AUTO, waits ready to arm and starts mission'''
        self.upload_simple_relhome_mission(items)
        self.set_current_waypoint(0, check_afterwards=False)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.send_cmd(mavutil.mavlink.MAV_CMD_MISSION_START)

    def send_do_reposition(self, target_lat, target_lon):
        '''send MAV_CMD_DO_REPOSITION with integer lat/lon (degE7) to move Rover in GUIDED mode'''
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p5=target_lat,
            p6=target_lon,
            p7=0,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
        )

    def JSON_sim_and_physics_time(self, physics):
        '''SITL time, and the physics time just before and after reading it'''
        before = physics.timestamp
        sim_time = self.get_sim_time()
        return sim_time, before, physics.timestamp

    def JSON_assert_elapsed_equal(self, start, end, tolerance=0.05):
        '''SITL time elapsed between two samples is the physics time elapsed

        Each physics read brackets its SITL read, which bounds the message
        latency exactly instead of allowing for it with a guessed tolerance.
        '''
        (sim0, before0, after0) = start
        (sim1, before1, after1) = end
        sim = sim1 - sim0
        self.progress("sim elapsed %f, physics elapsed %f-%f" % (sim, before1 - after0, after1 - before0))
        if not before1 - after0 - tolerance <= sim <= after1 - before0 + tolerance:
            raise NotAchievedException("SITL time drifted from physics time")

    def JSON_frame_rate(self, physics):
        '''median of the frame rates SITL sent in its last servo packets'''
        with physics.lock:
            rates = list(physics.frame_rates)
        rate = statistics.median(rates)
        self.progress("frame_rate median=%.1f min=%u max=%u over %u packets" % (rate, min(rates), max(rates), len(rates)))
        return rate

    def JSON_sim_wall_ratio(self, windows=5):
        '''median SITL/wall-clock rate over 1s windows

        A CI stall moves the mean of one long window, not the median of
        several short ones.
        '''
        ratios = []
        for i in range(windows):
            wall_start = time.time()
            sim_start = self.get_sim_time()
            time.sleep(1)
            ratios.append((self.get_sim_time() - sim_start) / (time.time() - wall_start))
        ratio = statistics.median(ratios)
        self.progress("sim/wall median=%f stdev=%f %s" % (ratio, statistics.stdev(ratios), ["%.2f" % r for r in ratios]))
        return ratio

    def tests(self):
        '''return list of all tests'''
        ret = super(AutoTestRover, self).tests()

        ret.extend([
            self.HIGH_LATENCY2,
            self.Gripper,
            self.SET_MESSAGE_INTERVAL,
            self.MESSAGE_INTERVAL_COMMAND_INT,
            self.REQUEST_MESSAGE,
            self.Button,
            self.DataFlashOverMAVLink,
            self.DataFlash,
            self.SetpointGlobalPos,
            self.SetpointGlobalVel,
            self.AccelCal,
            self.NoArmWithoutMissionItems,
            self.CompassPrearms,
        ])
        ret.extend(self.discovered_tests("Rover"))
        return ret

    def rc_defaults(self):
        ret = super(AutoTestRover, self).rc_defaults()
        ret[3] = 1500
        ret[8] = 1800
        return ret

    def initial_mode_switch_mode(self):
        return "MANUAL"

    def default_mode(self):
        return 'MANUAL'
