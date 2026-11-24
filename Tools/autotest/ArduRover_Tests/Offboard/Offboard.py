'''
Rover autotest: Offboard

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import PreconditionFailedException

from test_discovery import TestGroup
from test_discovery import testcase


class Offboard(TestGroup):
    # FIXME: test vehicle escape from outside inclusion zones to
    # inside inclusion zones (and inside exclusion zones to outside
    # exclusion zones)
    # FIXME: add test that a fence with edges that cross can't be uploaded
    # FIXME: add a test that fences enclose an area (e.g. all the points aren't the same value!
    @testcase(bucket="Rover")
    def Offboard(self, timeout=90):
        '''Test Offboard Control'''
        # rover-guided-mission.txt is in absolute coordinates, anchored
        # at the SITL startup location, and the run has to get all the
        # way round it and back home inside the timeout.  Starting from
        # wherever a previous test left the vehicle adds the drive out
        # to the mission and the drive home again at the end, and the
        # budget does not cover that:
        #     Offboard (Test Offboard Control) (Didn't complete)
        # with the mission on its last item, RTL, when time ran out.
        self.reboot_sitl()
        self.load_mission("rover-guided-mission.txt")
        self.wait_ready_to_arm(require_absolute=True)
        self.arm_vehicle()
        self.change_mode("AUTO")

        offboard_expected_duration = 10 # see mission file

        if self.mav.messages.get("SET_POSITION_TARGET_GLOBAL_INT", None):
            raise PreconditionFailedException("Already have SET_POSITION_TARGET_GLOBAL_INT")

        tstart = self.get_sim_time_cached()
        last_heartbeat_sent = 0
        got_ptgi = False
        magic_waypoint_tstart = 0
        magic_waypoint_tstop = 0
        while True:
            now = self.get_sim_time_cached()
            if now - last_heartbeat_sent > 1:
                last_heartbeat_sent = now
                self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                            0,
                                            0,
                                            0)

            if now - tstart > timeout:
                raise AutoTestTimeoutException("Didn't complete")
            magic_waypoint = 3
            mc = self.mav.recv_match(type=["MISSION_CURRENT", "STATUSTEXT"],
                                     blocking=False)
            if mc is not None:
                print("%s" % str(mc))
                if mc.get_type() == "STATUSTEXT":
                    if "Mission Complete" in mc.text:
                        break
                    continue
                if mc.seq == magic_waypoint:
                    print("At magic waypoint")
                    if magic_waypoint_tstart == 0:
                        magic_waypoint_tstart = self.get_sim_time_cached()
                    ptgi = self.mav.messages.get("POSITION_TARGET_GLOBAL_INT", None)
                    if ptgi is not None:
                        got_ptgi = True
                elif mc.seq > magic_waypoint:
                    if magic_waypoint_tstop == 0:
                        magic_waypoint_tstop = self.get_sim_time_cached()

        self.disarm_vehicle()
        offboard_duration = magic_waypoint_tstop - magic_waypoint_tstart
        if abs(offboard_duration - offboard_expected_duration) > 1:
            raise NotAchievedException("Did not stay in offboard control for correct time (want=%f got=%f)" %
                                       (offboard_expected_duration, offboard_duration))

        if not got_ptgi:
            raise NotAchievedException("Did not get ptgi message")
        print("pgti: %s" % str(ptgi))
