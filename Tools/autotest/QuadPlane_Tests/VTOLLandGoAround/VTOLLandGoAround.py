'''
QuadPlane autotest: VTOLLandGoAround

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class VTOLLandGoAround(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def VTOLLandGoAround(self):
        '''test MAV_CMD_DO_GO_AROUND is rejected early in a VTOL landing but
        accepted once paused or descending'''
        # Enable pause phase so it can also be checked
        self.set_parameter('Q_RTL_PAUSE_TIME', 10)

        # takeoff, fly out to the waypoint then approach a landing point back
        # near home, giving a full VTOL land approach. The DO_JUMP
        # returns us to the waypoint after each aborted landing so we can test
        # a go-around in several phases within a single flight.
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, 0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 700, 0, 40),
            # Small offset so its not converted into "land at current location"
            (mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND, 0.1, 0, 0),
            # jump back to the waypoint forever
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_JUMP,
                p1=2,
                p2=-1,
            ),
        ])

        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        def go_around(want_result):
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_GO_AROUND, want_result=want_result)

        # First landing attempt: the go-around should be rejected in the
        # approach and position phases, but accepted once we start the loiter
        # pause (before the abort_landing pause handling this was rejected).
        self.start_subtest("Go-around rejected during approach")
        self.wait_statustext('VTOL approach', timeout=180)
        go_around(mavutil.mavlink.MAV_RESULT_FAILED)

        self.start_subtest("Go-around rejected during position2")
        self.wait_statustext('VTOL position2 started', timeout=90)
        go_around(mavutil.mavlink.MAV_RESULT_FAILED)

        self.start_subtest("Go-around accepted during loiter pause")
        self.wait_statustext('Land pause started', timeout=60)
        go_around(mavutil.mavlink.MAV_RESULT_ACCEPTED)
        # the abort continues the mission (DO_JUMP) back to the waypoint
        self.wait_current_waypoint(2, timeout=30)

        # Second landing attempt: let the pause expire and abort the descent
        self.start_subtest("Go-around accepted during descent")
        self.wait_statustext('Land descend started', timeout=180)
        go_around(mavutil.mavlink.MAV_RESULT_ACCEPTED)
        self.wait_current_waypoint(2, timeout=30)

        # Third landing attempt: let it land for real
        self.start_subtest("Landing completes when not aborted")
        self.wait_disarmed(timeout=300)
