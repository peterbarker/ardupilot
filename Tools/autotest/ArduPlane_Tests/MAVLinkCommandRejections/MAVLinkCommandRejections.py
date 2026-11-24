'''
Plane autotest: MAVLinkCommandRejections

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAVLinkCommandRejections(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def MAVLinkCommandRejections(self):
        '''check Plane refuses commands it cannot act on'''
        here = self.get_location()
        lat = int(here.lat * 1e7)
        lng = int(here.lng * 1e7)
        DENIED = mavutil.mavlink.MAV_RESULT_DENIED
        FAILED = mavutil.mavlink.MAV_RESULT_FAILED

        self.change_mode('FBWA')

        self.start_subtest("DO_REPOSITION")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p5=lat,
            p6=lng,
            p7=100,
            frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            want_result=DENIED,
        )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p5=int(91e7),
            p6=lng,
            p7=100,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            want_result=DENIED,
        )
        # Plane takes a latitude and longitude of zero to mean "unset":
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p5=0,
            p6=0,
            p7=100,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            want_result=DENIED,
        )
        # not in GUIDED and not asked to change into it:
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p5=lat,
            p6=lng,
            p7=100,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            want_result=FAILED,
        )

        self.start_subtest("DO_CHANGE_ALTITUDE")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
            p1=100,
            p2=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            want_result=DENIED,
        )

        self.start_subtest("DO_CHANGE_SPEED outside AUTO and GUIDED")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            p1=mavutil.mavlink.SPEED_TYPE_AIRSPEED,
            p2=20,
            p3=-1,
            want_result=FAILED,
        )

        self.start_subtest("GUIDED_CHANGE commands outside GUIDED")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_SPEED,
            p1=mavutil.mavlink.SPEED_TYPE_AIRSPEED,
            p2=20,
            want_result=FAILED,
        )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            p7=100,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            want_result=FAILED,
        )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
            p1=mavutil.mavlink.HEADING_TYPE_HEADING,
            p2=90,
            want_result=FAILED,
        )

        self.change_mode('GUIDED')

        self.start_subtest("DO_CHANGE_SPEED with an unusable speed")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            p1=mavutil.mavlink.SPEED_TYPE_AIRSPEED,
            p2=1000,
            p3=-1,
            want_result=FAILED,
        )

        self.start_subtest("GUIDED_CHANGE_SPEED bad parameters")
        # only airspeed is supported:
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_SPEED,
            p1=mavutil.mavlink.SPEED_TYPE_GROUNDSPEED,
            p2=20,
            want_result=DENIED,
        )
        # outside AIRSPEED_MIN..AIRSPEED_MAX:
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_SPEED,
            p1=mavutil.mavlink.SPEED_TYPE_AIRSPEED,
            p2=1000,
            want_result=FAILED,
        )

        self.start_subtest("GUIDED_CHANGE_ALTITUDE bad parameters")
        # zero, and the -1 default, are refused:
        for alt in 0, -1:
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                p7=alt,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                want_result=DENIED,
            )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            p7=100,
            frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            want_result=DENIED,
        )

        self.start_subtest("GUIDED_CHANGE_HEADING bad parameters")
        for heading in -1, 360:
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
                p1=mavutil.mavlink.HEADING_TYPE_HEADING,
                p2=heading,
                want_result=DENIED,
            )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
            p1=3,  # not a HEADING_TYPE
            p2=90,
            want_result=DENIED,
        )

        self.start_subtest("MISSION_START with first/last items")
        self.run_cmd(mavutil.mavlink.MAV_CMD_MISSION_START, p1=1, want_result=DENIED)
        self.run_cmd(mavutil.mavlink.MAV_CMD_MISSION_START, p2=1, want_result=DENIED)
        self.assert_mode_is('GUIDED')

        self.start_subtest("DO_LAND_START without a landing sequence")
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, 50),
        ])
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_LAND_START, want_result=FAILED)

        self.start_subtest("DO_LAND_START when AUTO may not be entered from the GCS")
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, 50),
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_LAND_START),
            (mavutil.mavlink.MAV_CMD_NAV_LAND, 10, 0, 0),
        ])
        self.set_parameter("FLTMODE_GCSBLOCK", 1 << 9)  # AUTO
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_LAND_START, want_result=FAILED)
        self.assert_mode_is('GUIDED')
        self.set_parameter("FLTMODE_GCSBLOCK", 0)

        self.start_subtest("VTOL commands on a Plane which is not a QuadPlane")
        # COMMAND_LONG is converted to MAV_FRAME_LOCAL_OFFSET_NED:
        self.run_cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=10, want_result=FAILED)
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            p7=10,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            want_result=DENIED,
        )
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
            p1=mavutil.mavlink.MAV_VTOL_STATE_MC,
            want_result=FAILED,
        )
