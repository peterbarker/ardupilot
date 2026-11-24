'''
Plane autotest: MAV_CMD_DO_FLIGHTTERMINATION_unterminate

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_FLIGHTTERMINATION_unterminate(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def MAV_CMD_DO_FLIGHTTERMINATION_unterminate(self):
        '''unterminate a terminated vehicle'''
        self.set_parameters({
            "AFS_ENABLE": 1,
            "MAV_GCS_SYSID": self.mav.source_system,
            "AFS_TERM_ACTION": 42,
        })
        # AFS_TERMINATE magically set-and-saved by code:
        self.context_preserve_parameters(["AFS_TERMINATE"])
        self.takeoff(50, mode='TAKEOFF', timeout=200)

        # lock home to avoid alt messups
        original_home = self.home_position_as_location()
        self.set_home(original_home)

        self.context_collect('STATUSTEXT')

        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, p1=1)
        self.wait_disarmed()
        self.wait_text('Terminating due to GCS request', check_context=True)
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, p1=0)
        self.wait_disarmed()
        self.wait_text('Aborting termination due to GCS request', check_context=True)
        self.wait_ready_to_arm()  # please?
        self.arm_vehicle()

        self.fly_home_land_and_disarm()

        self.reboot_sitl()
