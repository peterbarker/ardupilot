'''
Copter autotest: PowerOff

AP_FLAKE8_CLEAN
'''

import time

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PowerOff(TestGroup):
    @testcase(bucket="CopterTests1c")
    def PowerOff(self):
        '''Test the shutdown command function'''
        self.set_parameter("SERIAL2_PROTOCOL", 2)
        self.reboot_sitl()
        self.wait_ready_to_arm()

        # a component sharing our system ID should be told the vehicle
        # is powering off:
        saved_mavfile_global = mavutil.mavfile_global
        component = mavutil.mavlink_connection(
            self.sitl_serial_endpoint(2),
            source_system=self.sysid_thismav(),
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        )
        mavutil.mavfile_global = saved_mavfile_global
        poweroff_commands = []

        def drain_component():
            while (m := component.recv_match()) is not None:
                if (m.get_type() == 'COMMAND_LONG' and
                        m.command == mavutil.mavlink.MAV_CMD_POWER_OFF_INITIATED):
                    poweroff_commands.append(m)

        try:
            # let the autopilot learn a route to the component:
            for i in range(3):
                component.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0)
                self.delay_sim_time(1, reason="autopilot to learn route to component")
                drain_component()

            self.context_collect('STATUSTEXT')
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                p1=2,  # REBOOT_SHUTDOWN_ACTION_SHUTDOWN
            )
            self.wait_statustext("Vehicle powering off", check_context=True)
            poweroff_start = self.get_sim_time_cached()

            # arming must be refused while the power off is pending,
            # even when forced:
            for p2 in 0, 2989:
                self.run_cmd(
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    p1=1,  # ARM
                    p2=p2,
                    want_result=mavutil.mavlink.MAV_RESULT_FAILED,
                )
            self.wait_statustext("Power off pending", check_context=True)

            # SITL should stay up for AP_POWEROFF_DELAY_MS, then exit:
            last_sim_time = poweroff_start
            tstart = time.time()
            # don't let pymavlink try to reconnect once SITL has gone:
            old_autoreconnect = self.mav.autoreconnect
            self.mav.autoreconnect = False
            connection_lost = False
            try:
                while self.sitl.isalive():
                    if time.time() - tstart > 60:
                        raise NotAchievedException("SITL did not power off")
                    self.drain_all_pexpects()
                    if connection_lost:
                        continue
                    try:
                        drain_component()
                        if self.mav.recv_match(type='SYSTEM_TIME', blocking=True, timeout=0.1) is not None:
                            last_sim_time = self.get_sim_time_cached()
                    except ConnectionError:
                        # SITL closes its sockets as it exits
                        connection_lost = True
            finally:
                self.mav.autoreconnect = old_autoreconnect
            if self.sitl.exitstatus != 0:
                raise NotAchievedException("SITL exited with status %s" % str(self.sitl.exitstatus))
            delay = last_sim_time - poweroff_start
            self.progress("Powered off %.1fs after power off initiated" % delay, send_statustext=False)
            if delay < 4 or delay > 6:
                raise NotAchievedException("Unexpected power off delay %.1fs" % delay)

            drain_component()
            if len(poweroff_commands) != 1:
                raise NotAchievedException(
                    "Expected one MAV_CMD_POWER_OFF_INITIATED, got %u" % len(poweroff_commands))
            if poweroff_commands[0].param1 != 0:
                raise NotAchievedException(
                    "Unexpected MAV_CMD_POWER_OFF_INITIATED param1 %f" % poweroff_commands[0].param1)
        finally:
            component.close()

        # restart the vehicle with default parameters:
        self.customise_SITL_commandline([], wipe=True)
