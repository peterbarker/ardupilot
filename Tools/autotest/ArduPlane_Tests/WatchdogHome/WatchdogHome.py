'''
Plane autotest: WatchdogHome

AP_FLAKE8_CLEAN
'''

import os
import signal

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class WatchdogHome(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def WatchdogHome(self):
        '''Ensure home is restored after watchdog reset'''
        if self.gdb:
            # we end up signalling the wrong process.  I think.
            # Probably need to have a "sitl_pid()" method to get the
            # ardupilot process's PID.
            self.progress("######## Skipping WatchdogHome test under GDB")
            return

        ex = None
        try:
            self.progress("Enabling watchdog")
            self.set_parameter("BRD_OPTIONS", 1 << 0)
            self.reboot_sitl()
            self.wait_ready_to_arm()
            self.progress("Explicitly setting home to a known location")
            orig_home = self.poll_home_position()
            new_home = orig_home
            new_home.latitude = new_home.latitude + 1000
            new_home.longitude = new_home.longitude + 2000
            new_home.altitude = new_home.altitude + 300000 # 300 metres
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                p5=new_home.latitude,
                p6=new_home.longitude,
                p7=new_home.altitude/1000.0, # mm => m
            )
            old_bootcount = self.get_parameter('STAT_BOOTCNT')
            self.progress("Forcing watchdog reset")
            os.kill(self.sitl.pid, signal.SIGALRM)
            self.detect_and_handle_reboot(old_bootcount)
            self.wait_statustext("WDG:")
            self.wait_statustext("IMU1 is using GPS")  # won't be come armable
            self.progress("Verifying home position")
            post_reboot_home = self.poll_home_position()
            delta = self.get_distance_int(new_home, post_reboot_home)
            max_delta = 1
            if delta > max_delta:
                raise NotAchievedException(
                    "New home not where it should be (dist=%f) (want=%s) (got=%s)" %
                    (delta, str(new_home), str(post_reboot_home)))
        except Exception as e:  # noqa: BLE001
            self.print_exception_caught(e)
            ex = e

        # The SIGALRM above makes SITL putenv("SITL_WATCHDOG_RESET=1")
        # and exec itself, and reboot_sitl() execs too, so that variable
        # - and the internal error every boot then raises because of it -
        # follows this process for as long as it lives.  Whatever runs
        # next on this SITL cannot become armable:
        #     PreArm: Internal errors 0x800 l:243 watchdog_rst
        # Rebooting does not shake it off; only a new process does.
        # customise_SITL_commandline() starts one, and tells the
        # framework it did, so the tidy-up afterwards knows the SITL has
        # been replaced - doing the stop/start by hand here left the
        # next test with no vehicle at all.
        self.customise_SITL_commandline([])

        if ex is not None:
            raise ex
