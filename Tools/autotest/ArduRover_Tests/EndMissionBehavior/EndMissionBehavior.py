'''
Rover autotest: EndMissionBehavior

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException

from test_discovery import TestGroup
from test_discovery import testcase


class EndMissionBehavior(TestGroup):
    @testcase(bucket="Rover")
    def EndMissionBehavior(self, timeout=60):
        '''Test end mission behavior'''
        self.context_push()

        self.load_mission("end-mission.txt")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.start_subtest("Test End Mission Behavior HOLD")
        self.context_collect("STATUSTEXT")
        self.change_mode("AUTO")
        self.wait_text("Mission Complete", check_context=True, wallclock_timeout=2)
        # On Hold we should just stop and don't update the navigation target anymore
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 15:
                raise AutoTestTimeoutException("Still getting POSITION_TARGET_GLOBAL_INT")
            m = self.mav.recv_match(type="POSITION_TARGET_GLOBAL_INT",
                                    blocking=True,
                                    timeout=10)
            if m is None:
                self.progress("No POSITION_TARGET_GLOBAL_INT received, all good !")
                break
        self.context_clear_collection("STATUSTEXT")
        self.change_mode("GUIDED")
        self.context_collect("STATUSTEXT")

        self.start_subtest("Test End Mission Behavior LOITER")
        self.set_parameter("MIS_DONE_BEHAVE", 1)
        self.change_mode("AUTO")
        self.wait_text("Mission Complete", check_context=True, wallclock_timeout=2)
        # On LOITER we should update the navigation target
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 15:
                raise AutoTestTimeoutException("Not getting POSITION_TARGET_GLOBAL_INT")
            m = self.mav.recv_match(type="POSITION_TARGET_GLOBAL_INT",
                                    blocking=True,
                                    timeout=5)
            if m is None:
                self.progress("No POSITION_TARGET_GLOBAL_INT received")
                continue
            else:
                if self.get_sim_time_cached() - tstart > 15:
                    self.progress("Got POSITION_TARGET_GLOBAL_INT, all good !")
                    break

        self.start_subtest("Test End Mission Behavior ACRO")
        self.set_parameter("MIS_DONE_BEHAVE", 2)
        # race conditions here to do with get_sim_time()
        # swallowing heartbeats means we have to be a little
        # circuitous when testing here:
        self.change_mode("GUIDED")
        self.send_cmd_do_set_mode('AUTO')
        self.wait_mode("ACRO")

        self.start_subtest("Test End Mission Behavior MANUAL")
        self.set_parameter("MIS_DONE_BEHAVE", 3)
        # race conditions here to do with get_sim_time()
        # swallowing heartbeats means we have to be a little
        # circuitous when testing here:
        self.change_mode("GUIDED")
        self.send_cmd_do_set_mode("AUTO")
        self.wait_mode("MANUAL")
        self.disarm_vehicle()

        self.context_pop()
        self.reboot_sitl()
