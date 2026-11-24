'''
Copter autotest: Parachute

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class Parachute(TestGroup):
    def _Parachute(self, command):
        '''Test Parachute Functionality using specific mavlink command'''
        self.set_rc(9, 1000)
        self.set_parameters({
            "CHUTE_ENABLED": 1,
            "CHUTE_TYPE": 10,
            "SERVO9_FUNCTION": 27,
            "SIM_PARA_ENABLE": 1,
            "SIM_PARA_PIN": 9,
            # not left over from a previous pass through this test;
            # RC9 low would otherwise disable the chute at each boot:
            "RC9_OPTION": 0,
        })

        self.progress("Test triggering parachute in mission")
        self.load_mission("copter_parachute_mission.txt")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.zero_throttle()
        self.arm_vehicle()
        self.change_mode('AUTO')
        self.set_rc(3, 1600)
        self.wait_statustext('BANG', timeout=60)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        self.progress("Test triggering with mavlink message")
        self.takeoff(20)
        self.context_collect('STATUSTEXT')
        command(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=2, # release
        )
        # check_context: the BANG can arrive while command() is still
        # draining messages awaiting its COMMAND_ACK
        self.wait_statustext('BANG', timeout=60, check_context=True)
        self.context_stop_collecting('STATUSTEXT')
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        self.progress("Testing three-position switch")
        self.set_parameter("RC9_OPTION", 23) # parachute 3pos

        self.progress("Test manual triggering")
        self.takeoff(20)
        self.set_rc(9, 2000)
        self.wait_statustext('BANG', timeout=60)
        self.set_rc(9, 1000)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        self.progress("Test mavlink triggering")
        self.takeoff(20)
        command(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=mavutil.mavlink.PARACHUTE_DISABLE,
        )
        ok = False
        try:
            self.wait_statustext('BANG', timeout=2)
        except AutoTestTimeoutException:
            ok = True
        if not ok:
            raise NotAchievedException("Disabled parachute fired")
        command(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=mavutil.mavlink.PARACHUTE_ENABLE,
        )
        ok = False
        try:
            self.wait_statustext('BANG', timeout=2)
        except AutoTestTimeoutException:
            ok = True
        if not ok:
            raise NotAchievedException("Enabled parachute fired")

        self.set_rc(9, 1000)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        # parachute should not fire if you go from disabled to release:
        self.takeoff(20)
        command(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=mavutil.mavlink.PARACHUTE_RELEASE,
        )
        ok = False
        try:
            self.wait_statustext('BANG', timeout=2)
        except AutoTestTimeoutException:
            ok = True
        if not ok:
            raise NotAchievedException("Parachute fired when going straight from disabled to release")

        # now enable then release parachute:
        command(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=mavutil.mavlink.PARACHUTE_ENABLE,
        )
        self.context_collect('STATUSTEXT')
        command(
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            p1=mavutil.mavlink.PARACHUTE_RELEASE,
        )
        # check_context: the BANG can arrive while command() is still
        # draining messages awaiting its COMMAND_ACK
        self.wait_statustext('BANG!  Parachute deployed', timeout=2, check_context=True)
        self.context_stop_collecting('STATUSTEXT')
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        self.context_push()
        self.progress("Crashing with 3pos switch in enable position")
        self.takeoff(40)
        self.set_rc(9, 1500)
        self.set_parameters({
            # both front motors; a single failed motor is compensated
            # well enough that the parachute's loss-of-control check
            # (sustained >30deg tilt error while descending) only
            # marginally triggers, and a failed diagonal pair leaves a
            # torque-balanced flat spin which never triggers it
            "SIM_ENGINE_FAIL": (1 << 0) | (1 << 2), # motors 1 and 3
        })
        self.wait_statustext('BANG!  Parachute deployed', timeout=60)
        self.set_rc(9, 1000)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        self.context_pop()

        # context so SIM_ENGINE_FAIL is reverted; leaving it set leaks
        # a dead motor into everything which follows
        self.context_push()
        self.progress("Crashing with 3pos switch in disable position")
        loiter_alt = 10
        self.takeoff(loiter_alt, mode='LOITER')
        self.set_rc(9, 1100)
        self.set_parameters({
            "SIM_ENGINE_FAIL": 1 << 1, # motor 2
        })
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + 5:
            m = self.assert_receive_message('STATUSTEXT')
            if m is None:
                continue
            if "BANG" in m.text:
                self.set_rc(9, 1000)
                self.reboot_sitl()
                raise NotAchievedException("Parachute deployed when disabled")
        self.set_rc(9, 1000)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        self.context_pop()

        # the parachute aux function has changed CHUTE_ENABLED's
        # value in RAM without saving it, so at this point the
        # parameter's value (0) does not match storage (1).  Set it
        # back to the known value this test runs with; the test
        # framework's end-of-test parameter revert then sees the
        # change it expects and restores the default properly, rather
        # than skipping the apparently-already-reverted parameter and
        # leaving a stale 1 in storage for the context-pop reboot to
        # resurrect - which would fail every subsequent test with
        # "PreArm: Chute has no relay".
        self.set_parameter("CHUTE_ENABLED", 1)

    @testcase(bucket="CopterTests2b")
    def Parachute(self):
        '''Test Parachute Functionality'''
        self._Parachute(self.run_cmd)
        self._Parachute(self.run_cmd_int)
