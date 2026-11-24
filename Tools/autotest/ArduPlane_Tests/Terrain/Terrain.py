'''
Plane autotest: Terrain

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class Terrain(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def Terrain(self):
        '''test AP_Terrain'''
        self.reboot_sitl()  # we know the terrain height at CMAC

        self.install_terrain_handlers_context()

        self.wait_ready_to_arm()
        loc = self.get_location()

        lng_int = int(loc.lng * 1e7)
        lat_int = int(loc.lat * 1e7)

        # FIXME: once we have a pre-populated terrain cache this
        # should require an instantly correct report to pass
        tstart = self.get_sim_time_cached()
        last_terrain_report_pending = -1
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 60:
                raise NotAchievedException("Did not get correct terrain report")

            self.mav.mav.terrain_check_send(lat_int, lng_int)

            report = self.assert_receive_message('TERRAIN_REPORT', timeout=60)
            self.progress(self.dump_message_verbose(report))
            if report.spacing != 0:
                break

            # we will keep trying to long as the number of pending
            # tiles is dropping:
            if last_terrain_report_pending == -1:
                last_terrain_report_pending = report.pending
            elif report.pending < last_terrain_report_pending:
                last_terrain_report_pending = report.pending
                tstart = now

            self.delay_sim_time(1, reason="terrain tile download")

        self.progress(self.dump_message_verbose(report))

        expected_terrain_height = 583.5
        if abs(report.terrain_height - expected_terrain_height) > 0.5:
            raise NotAchievedException("Expected terrain height=%f got=%f" %
                                       (expected_terrain_height, report.terrain_height))
