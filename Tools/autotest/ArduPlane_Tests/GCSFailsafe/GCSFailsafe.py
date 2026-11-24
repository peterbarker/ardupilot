'''
Plane autotest: GCSFailsafe

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GCSFailsafe(TestGroup):
    @testcase(bucket="PlaneTests1b", speedup=8)
    def GCSFailsafe(self):
        '''Ensure Long-Failsafe works on GCS loss'''
        self.start_subtest("Test Failsafe: RTL")
        self.load_sample_mission()
        self.set_parameters({
            "FS_GCS_ENABL": 1,
            "FS_LONG_ACTN": 1,
            "RTL_AUTOLAND": 1,
            "MAV_GCS_SYSID": self.mav.source_system,
        })
        self.takeoff()
        self.change_mode('LOITER')
        self.progress("Disconnecting GCS")
        self.set_heartbeat_rate(0)
        self.wait_mode("RTL", timeout=10)
        self.set_heartbeat_rate(self.speedup)
        self.end_subtest("Completed RTL Failsafe test")

        self.start_subtest("Test Failsafe: Glide")
        self.set_parameters({
            "FS_LONG_ACTN": 2,
        })
        self.change_mode('AUTO')
        self.progress("Disconnecting GCS")
        self.set_heartbeat_rate(0)
        self.wait_mode(27, timeout=10)  # Glide, need pymavlink support to use text name
        self.set_heartbeat_rate(self.speedup)
        self.end_subtest("Completed Glide Failsafe test")

        self.start_subtest("Test Failsafe: Deploy Parachute")
        self.load_mission("plane-parachute-mission.txt")
        self.set_current_waypoint(1)
        self.setup_simulated_parachute({"FS_LONG_ACTN": 3})
        self.change_mode("AUTO")
        self.progress("Disconnecting GCS")
        self.set_heartbeat_rate(0)
        self.wait_statustext("BANG", timeout=60)
        self.set_heartbeat_rate(self.speedup)
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        self.end_subtest("Completed Parachute Failsafe test")
