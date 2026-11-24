'''
Plane autotest: MAV_CMD_GUIDED_CHANGE_ALTITUDE

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_GUIDED_CHANGE_ALTITUDE(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def MAV_CMD_GUIDED_CHANGE_ALTITUDE(self):
        '''test handling of MAV_CMD_GUIDED_CHANGE_ALTITUDE'''
        target_alt = 750
        # this location is chosen to be fairly flat, but at a different terrain height to home
        higher_ground = Location(-35.35465024, 149.13974996, target_alt, AltFrame.ABSOLUTE)
        self.install_terrain_handlers_context()
        self.start_subtest("set home relative altitude")
        self.takeoff(30, relative=True)
        self.set_rc(3, 1500)
        self.change_mode('GUIDED')

        # remember home
        home_loc = self.home_position_as_location()
        height_diff = target_alt - home_loc.get_alt_m(AltFrame.ABSOLUTE)

        # fly to higher ground
        self.send_do_reposition(higher_ground)
        self.wait_location(
            higher_ground,
            accuracy=200,
            timeout=600,
            height_accuracy=10,
        )

        for alt in 50, 70:
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                p7=alt+height_diff,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            )
            self.wait_altitude(target_alt+alt-1, target_alt+alt+1, timeout=30, minimum_duration=10, relative=False)

        # test for #24535
        self.start_subtest("switch to loiter and resume guided maintains home relative altitude")
        self.change_mode('LOITER')
        self.delay_sim_time(1, reason="loiter to stabilise altitude")
        self.change_mode('GUIDED')
        self.wait_altitude(
            height_diff+alt-3,  # NOTE: reuse of alt from above loop!
            height_diff+alt+3,
            minimum_duration=10,
            timeout=30,
            relative=True,
        )
        # test that this works if switching between RELATIVE (HOME) and GLOBAL(AMSL)
        self.start_subtest("set global/AMSL altitude, switch to loiter and resume guided")
        alt = target_alt+30
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            p7=alt,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
        )
        self.wait_altitude(alt-3, alt+3, timeout=30, relative=False, minimum_duration=10)

        # let it settle so LOITER captures a constant altitude
        self.delay_sim_time(10, reason="altitude to settle for LOITER capture")

        # now switch to LOITER then back to GUIDED
        self.change_mode('LOITER')
        self.delay_sim_time(5, reason="loiter to capture altitude")
        self.change_mode('GUIDED')
        self.wait_altitude(
            alt-5,  # NOTE: reuse of alt from above CHANGE_ALTITUDE
            alt+5,
            minimum_duration=10,
            timeout=30,
            relative=False,
        )

        # test that this works if switching between RELATIVE (HOME) and terrain
        self.start_subtest("set terrain altitude, switch to loiter and resume guided")
        self.change_mode('GUIDED')
        alt = 100
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            p7=alt,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
        )
        self.wait_altitude(
            alt-10,  # NOTE: reuse of alt from abovE
            alt+10,  # use a 10m buffer as the plane needs to go up and down a bit to maintain terrain distance
            minimum_duration=10,
            timeout=30,
            relative=False,
            altitude_source="TERRAIN_REPORT.current_height"
        )

        alt = 150
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            p7=alt,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
        )
        self.wait_altitude(
            alt-10,  # NOTE: reuse of alt from abovE
            alt+10,  # use a 10m buffer as the plane needs to go up and down a bit to maintain terrain distance
            minimum_duration=10,
            timeout=30,
            relative=False,
            altitude_source="TERRAIN_REPORT.current_height"
        )
        self.delay_sim_time(30, reason="terrain altitude to stabilise")

        self.change_mode('LOITER')
        self.delay_sim_time(1, reason="loiter to capture terrain altitude")
        self.change_mode('GUIDED')
        self.wait_altitude(
            alt-5,  # NOTE: reuse of alt from abovE
            alt+5,  # use a 5m buffer as the plane needs to go up and down a bit to maintain terrain distance
            minimum_duration=10,
            timeout=30,
            relative=False,
            altitude_source="TERRAIN_REPORT.current_height"
        )

        self.start_subtest("test flyto with relative alt")
        dest = higher_ground.copy()
        dest_alt = height_diff + 100
        dest.set_alt_m(dest_alt, AltFrame.ABOVE_HOME)
        self.progress("dest alt=%.1f" % dest_alt)
        self.send_do_reposition(dest)
        self.wait_location(
            dest,
            accuracy=200,
            timeout=600,
            height_accuracy=None,  # dest's altitude is relative; alt checked below
        )
        self.wait_altitude(
            dest_alt-5,
            dest_alt+5,
            minimum_duration=10,
            timeout=30,
            relative=True
        )

        self.start_subtest("test flyto with absolute alt")
        dest = higher_ground.copy()
        dest_alt = target_alt + 60
        dest.set_alt_m(dest_alt, AltFrame.ABSOLUTE)
        self.progress("dest alt=%.1f" % dest_alt)
        self.send_do_reposition(dest)
        self.wait_location(
            dest,
            accuracy=200,
            timeout=600,
            height_accuracy=10,
        )
        self.wait_altitude(
            dest_alt-5,
            dest_alt+5,
            minimum_duration=10,
            timeout=30,
            relative=False
        )

        self.start_subtest("test flyto with terrain alt")
        dest = higher_ground.copy()
        dest_alt = 130
        dest.set_alt_m(dest_alt, AltFrame.ABOVE_TERRAIN)
        self.progress("dest alt=%.1f" % dest_alt)
        self.send_do_reposition(dest)
        self.wait_location(
            dest,
            accuracy=200,
            timeout=600,
            height_accuracy=None,  # dest's altitude is above-terrain; alt checked below
        )
        self.wait_altitude(
            dest_alt-10,
            dest_alt+10,
            minimum_duration=10,
            timeout=30,
            relative=False,
            altitude_source="TERRAIN_REPORT.current_height"
        )

        self.delay_sim_time(5, reason="terrain altitude to stabilise before landing")
        self.fly_home_land_and_disarm()
